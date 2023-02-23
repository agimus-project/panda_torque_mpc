#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>

#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-fast.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>

#include "common.h"

namespace panda_torque_mpc
{

    namespace pin = pinocchio;

    struct TsidConfig
    {
        // task space gains
        double kp_ee = 1.0;
        double kp_posture = 1.0;

        // task weights
        double w_ee = 1.0;
        double w_posture = 1e-3;
        double w_torque_bounds = 1.0;
        double w_joint_bounds = 1.0;

        // other parameters
        double tau_limit_scale = 0.5;
        double v_limit_scale = 0.2;

        std::string ee_frame_name = "panda_link8";
        Vector6d ee_task_mask = (Vector6d() << 1, 1, 1, 1, 1, 1).finished();
    };

    class TsidManipulatorReaching
    {
    public:
        typedef tsid::robots::RobotWrapper RobotWrapper;
        typedef tsid::InverseDynamicsFormulationAccForce IDFormulation;
        typedef tsid::tasks::TaskSE3Equality TaskSE3Equality;
        typedef tsid::tasks::TaskJointPosture TaskJointPosture;
        typedef tsid::tasks::TaskActuationBounds TaskActuationBounds;
        typedef tsid::tasks::TaskJointBounds TaskJointBounds;
        typedef tsid::solvers::SolverHQuadProgFast SolverHQuadProgFast;

        TsidManipulatorReaching()
        {
            // dummy constructor necessary to use this class as a member variable directly
        }

        TsidManipulatorReaching(std::string _model_path, const TsidConfig &_conf) : conf_(_conf)
        {
            /**
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             * Do NOT use:
             * RobotWrapper(pin::Model model, true)
             *
             * This constructor currently (23/02/23) harcodes the assumption of the presence of a free-flyer joint
             * which produces an inconsistent problem down the line
             *
             */
            tsid_robot_ = std::make_unique<RobotWrapper>(_model_path, std::vector<std::string>(), true);
            // std::cout << "tsid_robot_->nq(), tsid_robot_->nv(), tsid_robot_->na():\n"
            //           << tsid_robot_->nq() << ", " << tsid_robot_->nv() << ", " << tsid_robot_->na() << std::endl;
            formulation_ = std::make_unique<IDFormulation>("tsid", *tsid_robot_, false);

            // 1) posture task
            postureTask_ = std::make_unique<TaskJointPosture>("task-posture", *tsid_robot_);
            postureTask_->Kp(conf_.kp_posture * Vector7d::Ones());
            postureTask_->Kd(2.0 * sqrt(conf_.kp_posture) * Vector7d::Ones());
            formulation_->addMotionTask(*postureTask_, conf_.w_posture, 1, 0.0);

            // 2) EE tracking task
            eeTask_ = std::make_unique<TaskSE3Equality>("task-ee", *tsid_robot_, conf_.ee_frame_name);
            eeTask_->Kp(conf_.kp_ee * Vector6d::Ones());
            eeTask_->Kd(2.0 * sqrt(conf_.kp_ee) * Vector6d::Ones());
            eeTask_->setMask(conf_.ee_task_mask);
            // TODO: check what this does exactly
            eeTask_->useLocalFrame(false);
            auto ee_id = tsid_robot_->model().getFrameId(conf_.ee_frame_name);
            pin::SE3 H_ee_ref = tsid_robot_->framePosition(formulation_->data(), ee_id);
            formulation_->addMotionTask(*eeTask_, conf_.w_ee, 1, 0.0);

            // 3) Actuation bound Constraint
            // TODO: read from param server
            actuationBoundsTask_ = std::make_unique<TaskActuationBounds>("task-actuation-bounds", *tsid_robot_);
            Vector7d tau_max = conf_.tau_limit_scale * tsid_robot_->model().effortLimit;
            Vector7d tau_min = -tau_max;
            std::cout << tau_max.transpose() << std::endl;
            actuationBoundsTask_->setBounds(tau_min, tau_max);
            if (conf_.w_torque_bounds > 0.0)
                formulation_->addActuationTask(*actuationBoundsTask_, conf_.w_torque_bounds, 0, 0.0);

            // 4) Vel constraint is actually implemented as an acceleration constraint:
            // ddq_max_due_to_vel = (v_ub - va)/dt;
            //  -> trim the acceleration on a given joint so that integrating it for dt does not go beyond vel limit
            double dt_margin = 2e-3; // margin before joint limit collision
            jointBoundsTask_ = std::make_unique<TaskJointBounds>("task-joint-bounds", *tsid_robot_, dt_margin);
            Vector7d v_max = conf_.v_limit_scale * tsid_robot_->model().velocityLimit;
            Vector7d v_min = -v_max;
            std::cout << v_max.transpose() << std::endl;
            jointBoundsTask_->setVelocityBounds(v_min, v_max);
            if (conf_.w_joint_bounds > 0.0)
                formulation_->addMotionTask(*jointBoundsTask_, conf_.w_joint_bounds, 0, 0.0);

            // SOLVER
            solver_qp_ = std::make_unique<SolverHQuadProgFast>("qp solver");
            solver_qp_->resize(formulation_->nVar(), formulation_->nEq(), formulation_->nIn());
            // std::cout << "formulation_->nVar(), formulation_->nEq(), formulation_->nIn():\n"
            //           << formulation_->nVar() << ", " << formulation_->nEq() << ", " << formulation_->nIn() << std::endl;
        }

        void setPostureRef(const Vector7d &q_ref)
        {
            auto trajPosture = tsid::trajectories::TrajectoryEuclidianConstant("traj_joint", q_ref);
            postureTask_->setReference(trajPosture.computeNext());
        }

        void setEERef(const pin::SE3 &x_r, const pin::Motion &dx_r, const pin::Motion &ddx_r)
        {
            // EE tracking
            tsid::trajectories::TrajectorySample sampleEE;
            // pos = [posi, R_flattened]
            Eigen::Matrix<double, 12, 1> pos;
            pos.head<3>() = x_r.translation();
            Eigen::MatrixXd R_flattened = x_r.rotation();
            R_flattened.resize(9, 1);
            pos.tail<9>() = R_flattened;
            sampleEE.setValue(pos);
            sampleEE.setDerivative(dx_r.toVector());
            sampleEE.setSecondDerivative(ddx_r.toVector());

            eeTask_->setReference(sampleEE);
        }

        void solve(const Vector7d &q_m, const Vector7d &dq_m)
        {
            // time is only useful in computeProblemData when the model has contact switches
            // --> not our case
            double time = 0.0;
            auto HQPData = formulation_->computeProblemData(time, q_m, dq_m);

            auto sol = solver_qp_->solve(HQPData);
            // std::cout << "Solver status: " << sol.status << std::endl;
            // std::cout << "Solver iterations: " << sol.iterations << std::endl;

            if (sol.status != 0)
            {
                std::cout << "QP could not be solved, error code: " << sol.status << std::endl;
            }

            ddq_d_ = formulation_->getAccelerations(sol);
            tau_d_ = formulation_->getActuatorForces(sol);
        }

        Eigen::VectorXd getAccelerations() { return ddq_d_; }
        Eigen::VectorXd getTorques() { return tau_d_; }

        TsidConfig conf_;

        // TSID objects we need to access in the controller body
        // All tasks need to be member variable otherwise a segfault
        // happens when calling computeProblemData
        std::unique_ptr<RobotWrapper> tsid_robot_;
        std::unique_ptr<IDFormulation> formulation_;
        std::unique_ptr<TaskSE3Equality> eeTask_;
        std::unique_ptr<TaskJointPosture> postureTask_;
        std::unique_ptr<TaskActuationBounds> actuationBoundsTask_;
        std::unique_ptr<TaskJointBounds> jointBoundsTask_;
        std::unique_ptr<SolverHQuadProgFast> solver_qp_;

        // optimization problem results
        Eigen::VectorXd ddq_d_;
        Eigen::VectorXd tau_d_;
    };

} // namespace panda_torque_mpc