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

    class TsidReaching
    {
    public:
        typedef tsid::robots::RobotWrapper RobotWrapper;
        typedef tsid::InverseDynamicsFormulationAccForce IDFormulation;
        typedef tsid::tasks::TaskSE3Equality TaskSE3Equality;
        typedef tsid::tasks::TaskJointPosture TaskJointPosture;
        typedef tsid::solvers::SolverHQuadProgFast SolverHQuadProgFast;

        TsidReaching()
        {
            // dummy constructor necessary to use this class as a member variable directly
        }

        TsidReaching(const pin::Model &_model_pin, std::string _ee_frame_pin,
                     double Kp, double Kd, double w_posture)
        {

            tsid_robot_ = std::make_unique<RobotWrapper>(_model_pin, true);
            formulation_ = std::make_unique<IDFormulation>("tsid", *tsid_robot_, true);
            formulation_->computeProblemData(0.0, Vector7d::Zero(), Vector7d::Zero());

            // 1) EE tracking task
            eeTask_ = std::make_unique<TaskSE3Equality>("task-ee", *tsid_robot_, _ee_frame_pin);
            eeTask_->Kp(Kp * Eigen::Matrix<double, 6, 1>::Ones());
            eeTask_->Kd(Kd * Eigen::Matrix<double, 6, 1>::Ones());
            Eigen::Matrix<double, 6, 1> ee_task_mask = Eigen::Matrix<double, 6, 1>::Ones();
            eeTask_->setMask(ee_task_mask);
            // TODO: check what this does exactly
            eeTask_->useLocalFrame(false);
            double w_ee = 1.0;
            formulation_->addMotionTask(*eeTask_, w_ee, 1, 0.0);

            // 2) posture task
            postureTask_ = std::make_unique<TaskJointPosture>("task-posture", *tsid_robot_);
            postureTask_->Kp(Kp * Eigen::Matrix<double, 7, 1>::Ones());
            postureTask_->Kd(Kd * Eigen::Matrix<double, 7, 1>::Ones());
            formulation_->addMotionTask(*postureTask_, w_posture, 1, 0.0);

            // 3) Actuation bound Constraint
            // TODO: read from param server
            double tau_limit_scale = 0.5;
            Vector7d tau_max = tau_limit_scale * _model_pin.effortLimit;
            Vector7d tau_min = -tau_max;
            tsid::tasks::TaskActuationBounds actuationBoundsTask("task-actuation-bounds", *tsid_robot_);
            actuationBoundsTask.setBounds(tau_min, tau_max);
            double w_torque_bounds = 1.0;
            if (w_torque_bounds > 0.0)
                formulation_->addActuationTask(actuationBoundsTask, w_torque_bounds, 0, 0.0);

            // 4) Vel constraint is actually implemented as an acceleration constraint:
            // ddq_max_due_to_vel = (v_ub - va)/dt;
            //  -> trim the acceleration on a given joint so that integrating it for dt does not go beyond vel limit
            double dt_margin = 2e-3; // margin before joint limit collision
            tsid::tasks::TaskJointBounds jointBoundsTask("task-joint-bounds", *tsid_robot_, dt_margin);
            double v_limit_scale = 0.5;
            Vector7d v_max = v_limit_scale * _model_pin.velocityLimit;
            Vector7d v_min = -v_max;
            jointBoundsTask.setVelocityBounds(v_min, v_max);
            double w_joint_bounds = 1.0;
            if (w_joint_bounds > 0.0)
                formulation_->addMotionTask(jointBoundsTask, w_joint_bounds, 0, 0.0);

            // SOLVER
            solver_qp_ = std::make_unique<SolverHQuadProgFast>("qp solver");
            solver_qp_->resize(formulation_->nVar(), formulation_->nEq(), formulation_->nIn());
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

        void solve(const Vector7d& q_m, const Vector7d& dq_m)
        {
            // time is only useful in computeProblemData when we have contact switches
            double time = 0.0;
            /**
            // DEBUG:
            gzserver: /usr/include/eigen3/Eigen/src/Core/util/XprHelper.h:113:
            Eigen::internal::variable_if_dynamic<T, Value>::variable_if_dynamic(T) [with T = long int; int Value = 3]: Assertion `v == T(Value)' failed.

            Means that a mismatch is happening between a compile time Eigen matrix size definition and a runtime usage
            */
            auto HQPData = formulation_->computeProblemData(time, q_m, dq_m);

            auto sol = solver_qp_->solve(HQPData);
            if (sol.status != 0)
            {
                // ROS_INFO_STREAM("QP problem could not be solved! Error code: " << sol.status);
            }

            ddq_d_ = formulation_->getAccelerations(sol);
            tau_d_ = formulation_->getActuatorForces(sol);
        }

        Vector7d getAccelerations() {return ddq_d_;}
        Vector7d getTorques() {return tau_d_;}

        // TSID objects we need to access in the controller body
        std::unique_ptr<RobotWrapper> tsid_robot_;
        std::unique_ptr<IDFormulation> formulation_;
        std::unique_ptr<TaskSE3Equality> eeTask_;
        std::unique_ptr<TaskJointPosture> postureTask_;
        std::unique_ptr<SolverHQuadProgFast> solver_qp_;

        // optimization problem results
        Vector7d ddq_d_; 
        Vector7d tau_d_; 
    };

} // namespace panda_torque_mpc