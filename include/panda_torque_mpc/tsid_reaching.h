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




namespace panda_torque_mpc {

namespace pin = pinocchio;

class TsidReaching 
{
    public:
    // Most tsid objects do not have default constructors
    // --> need to be put in the initializer list
    TsidReaching(pin::Model _model_pin, std::string _ee_frame_pin, 
                 double Kp, double Kd, double w_posture):
        tsid_robot_(_model_pin, true),
        formulation_("tsid", tsid_robot_, true),
        eeTask_("task-ee", tsid_robot_, _ee_frame_pin),
        postureTask_("task-posture", tsid_robot_),
        solver_tsid_("qp solver")
    {

        // 1) EE tracking task
        eeTask_.Kp(Kp*Eigen::Matrix<double, 6, 1>::Ones());
        eeTask_.Kd(Kd*Eigen::Matrix<double, 6, 1>::Ones());
        Eigen::VectorXd ee_task_mask =  Eigen::Matrix<double, 6, 1>::Ones();
        eeTask_.setMask(ee_task_mask);
        // TODO: check what this does exactly
        eeTask_.useLocalFrame(false);
        double w_ee = 1.0;
        formulation_.addMotionTask(eeTask_, w_ee, 1, 0.0);

        // 2) posture task
        postureTask_.Kp(Kp*Eigen::Matrix<double, 7, 1>::Ones());
        postureTask_.Kd(Kd*Eigen::Matrix<double, 7, 1>::Ones());
        formulation_.addMotionTask(postureTask_, w_posture, 1, 0.0);

        // 3) Actuation bound Constraint
        // TODO: read from param server
        double tau_limit_scale = 0.5;
        Vector7d tau_max = tau_limit_scale*_model_pin.effortLimit;
        Vector7d tau_min = -tau_max;
        tsid::tasks::TaskActuationBounds actuationBoundsTask("task-actuation-bounds", tsid_robot_);
        actuationBoundsTask.setBounds(tau_min, tau_max);
        double w_torque_bounds = 1.0;
        if(w_torque_bounds > 0.0)
            formulation_.addActuationTask(actuationBoundsTask, w_torque_bounds, 0, 0.0);

        // 4) Vel constraint is actually implemented as an acceleration constraint: 
        // ddq_max_due_to_vel = (v_ub - va)/dt;
        //  -> trim the acceleration on a given joint so that integrating it for dt does not go beyond vel limit
        double dt_margin = 2e-3;  // margin before joint limit collision
        tsid::tasks::TaskJointBounds jointBoundsTask("task-joint-bounds", tsid_robot_, dt_margin);
        double v_limit_scale = 0.5;
        Vector7d v_max = v_limit_scale*_model_pin.velocityLimit;
        Vector7d v_min = -v_max;
        jointBoundsTask.setVelocityBounds(v_min, v_max);
        double w_joint_bounds = 1.0;
        if(w_joint_bounds > 0.0)
            formulation_.addMotionTask(jointBoundsTask, w_joint_bounds, 0, 0.0);
        
        // SOLVER
        solver_tsid_.resize(formulation_.nVar(), formulation_.nEq(), formulation_.nIn());

    }

    // TSID objects we need to access in the controller body
    tsid::robots::RobotWrapper tsid_robot_;
    tsid::InverseDynamicsFormulationAccForce formulation_;
    tsid::tasks::TaskSE3Equality eeTask_;
    tsid::tasks::TaskJointPosture postureTask_;
    tsid::solvers::SolverHQuadProgFast solver_tsid_;

};


} // namespace panda_torque_mpc