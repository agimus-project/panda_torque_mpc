

#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include <cmath>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/constraints/constraint-manager.hpp>

#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/constraints/residual.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/activations/quadratic.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>

#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/control-gravity.hpp>



#include "panda_torque_mpc/crocoddyl_reaching.h"

namespace panda_torque_mpc
{
    CrocoddylReaching::CrocoddylReaching(const pin::Model model_pin, const boost::shared_ptr<pin::GeometryModel>& collision_model ,
        CrocoddylConfig config,  TargetsConfig targ_config) :
    config_(config), collision_model_(collision_model), targ_config_(targ_config)
    
    {

        const std::size_t end_effector_frame_id = model_pin.getFrameId(config.ee_frame_name);

        std::cout << "Creating state, actuation and IAMs... " << std::endl;
        auto state = boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(model_pin));
        auto actuation = boost::make_shared<crocoddyl::ActuationModelFull>(state);

        Eigen::Matrix<double, 14, 1> diag_x_reg_running; diag_x_reg_running << config.diag_q_reg_running, config.diag_v_reg_running;
        Eigen::Matrix<double, 14, 1> diag_x_reg_terminal = diag_x_reg_running;

        // !! Send an error if not enough tasks set?
        goal_translation_set_ = false;
        goal_placement_set_ = false;
        posture_set_ = false;
        Eigen::Vector3d dummy_translation_reference = Eigen::Vector3d::Zero();
        pin::SE3 dummy_placement_reference = pin::SE3::Identity();
        Eigen::Matrix<double, 14, 1> x0_dummy = Eigen::Matrix<double, 14, 1>::Zero();

        // Cost names
        cost_translation_name_ = "translation_cost";
        cost_placement_name_ = "placement_cost";
        cost_velocity_name_ = "velocity_cost";
        cost_state_reg_name_ = "state_reg";
        cost_ctrl_reg_name_ = "ctrl_reg";

        // Collision constraints
        auto runningConstraintModelManager =  boost::make_shared<crocoddyl::ConstraintModelManager>(state);
        auto terminalConstraintModelManager =  boost::make_shared<crocoddyl::ConstraintModelManager>(state);
        Eigen::VectorXd lb_col(1), ub_col(1);
        lb_col << config.collision_safety_margin;
        ub_col << std::numeric_limits<double>::infinity();

        for (int col_idx = 0; col_idx < collision_model->collisionPairs.size(); col_idx++)
        {

            auto obstacle_distance_residual = boost::make_shared<colmpc::ResidualDistanceCollision>
                (colmpc::ResidualDistanceCollision(state, 7, collision_model, col_idx));
            auto constraint = boost::make_shared<crocoddyl::ConstraintModelResidual>(
                state,
                obstacle_distance_residual,
                lb_col,
                ub_col
            );
            std::string running_constraint_name = "col" + std::to_string(col_idx);
            std::string terminal_constraint_name = "col_term" + std::to_string(col_idx);
            runningConstraintModelManager->addConstraint(running_constraint_name, constraint);
            terminalConstraintModelManager->addConstraint(terminal_constraint_name, constraint);

        }

        // Frame translation
        // auto frame_translation_cost = boost::make_shared<crocoddyl::CostModelResidual>(
        //     state,
        //     boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state, end_effector_frame_id, dummy_translation_reference, actuation->get_nu()));

        // // Frame placement
        // auto frame_placement_cost = boost::make_shared<crocoddyl::CostModelResidual>(
        //     state,
        //     boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state, end_effector_frame_id, dummy_placement_reference, actuation->get_nu()));

        // // Frame velocity
        Eigen::Matrix<double, 6, 1> frame_velocity_reference = Eigen::Matrix<double, 6, 1>::Zero();
        // auto frame_velocity_cost = boost::make_shared<crocoddyl::CostModelResidual>(
        //     state,
        //     boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(config.diag_frame_vel),
        //     boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(state, end_effector_frame_id, pin::Motion(frame_velocity_reference), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, actuation->get_nu()));

        auto running_IAMs = std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>(config.T);


        for (int i = 0; i < config.T; i++)
        {
            // velocity reg
            auto frame_velocity_cost = boost::make_shared<crocoddyl::CostModelResidual>(
            state,
            boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(config.diag_frame_vel),
            boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(state, end_effector_frame_id, pin::Motion(frame_velocity_reference), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, actuation->get_nu()));

            // Frame translation
            auto frame_translation_cost = boost::make_shared<crocoddyl::CostModelResidual>(
                state,
                boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state, end_effector_frame_id, dummy_translation_reference, actuation->get_nu()));

            // Frame placement
            auto frame_placement_cost = boost::make_shared<crocoddyl::CostModelResidual>(
                state,
                boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state, end_effector_frame_id, dummy_placement_reference, actuation->get_nu()));
            // State reg
            auto state_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(
                state,
                boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(diag_x_reg_running),
                boost::make_shared<crocoddyl::ResidualModelState>(state, x0_dummy, actuation->get_nu()));

            // Ctrl reg
            auto ctrl_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(
                state,
                boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(config.diag_u_reg_running),
                boost::make_shared<crocoddyl::ResidualModelControlGrav>(state, actuation->get_nu()));

            auto runningCostModel = boost::make_shared<crocoddyl::CostModelSum>(state);
            runningCostModel.get()->addCost(cost_state_reg_name_,   state_reg_cost, config.w_x_reg_running);
            runningCostModel.get()->addCost(cost_ctrl_reg_name_,    ctrl_reg_cost, config.w_u_reg_running);
            runningCostModel.get()->addCost(cost_translation_name_, frame_translation_cost, config.w_frame_running); // TODO: weight schedule
            runningCostModel.get()->addCost(cost_placement_name_,   frame_placement_cost, config.w_frame_running); // TODO: weight schedule
            runningCostModel.get()->addCost(cost_velocity_name_,    frame_velocity_cost, config.w_frame_vel_running); // TODO: weight schedule
            
            auto running_DAM = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, runningCostModel, runningConstraintModelManager);
            running_DAM->set_armature(config.armature);

            // Deactivate goal cost by default until a proper reference is set
            running_DAM->get_costs()->changeCostStatus(cost_state_reg_name_, false);
            running_DAM->get_costs()->changeCostStatus(cost_translation_name_, false);
            running_DAM->get_costs()->changeCostStatus(cost_placement_name_, false);

            running_IAMs[i] = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(running_DAM, config.dt_ocp);
        }

        // velocity reg
        auto frame_velocity_terminal_cost = boost::make_shared<crocoddyl::CostModelResidual>(
        state,
        boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(config.diag_frame_vel),
        boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(state, end_effector_frame_id, pin::Motion(frame_velocity_reference), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, actuation->get_nu()));

        // Frame translation
        auto frame_translation_terminal_cost = boost::make_shared<crocoddyl::CostModelResidual>(
            state,
            boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state, end_effector_frame_id, dummy_translation_reference, actuation->get_nu()));

        // Frame placement
        auto frame_placement_terminal_cost = boost::make_shared<crocoddyl::CostModelResidual>(
            state,
            boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state, end_effector_frame_id, dummy_placement_reference, actuation->get_nu()));

        auto terminalCostModel = boost::make_shared<crocoddyl::CostModelSum>(state);
        // State reg
        auto state_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(
            state,
            boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(diag_x_reg_terminal),
            boost::make_shared<crocoddyl::ResidualModelState>(state, x0_dummy, actuation->get_nu()));

        // terminal gains have to be multiplied by dt in order to be up to scale with running costs
        terminalCostModel.get()->addCost(cost_state_reg_name_,   state_reg_cost,         config.w_x_reg_terminal*config.dt_ocp);
        terminalCostModel.get()->addCost(cost_translation_name_, frame_translation_terminal_cost, config.w_frame_terminal*config.dt_ocp);
        terminalCostModel.get()->addCost(cost_placement_name_,   frame_placement_terminal_cost,   config.w_frame_terminal*config.dt_ocp);
        terminalCostModel.get()->addCost(cost_velocity_name_,    frame_velocity_terminal_cost,    config.w_frame_vel_terminal*config.dt_ocp);

        auto terminal_DAM = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, terminalCostModel, terminalConstraintModelManager);
        terminal_DAM->set_armature(config.armature);

        // Deactivate goal cost by default until a proper reference is set
        terminal_DAM->get_costs()->changeCostStatus(cost_state_reg_name_, false);
        terminal_DAM->get_costs()->changeCostStatus(cost_translation_name_, false);
        terminal_DAM->get_costs()->changeCostStatus(cost_placement_name_, false);

        auto terminal_IAM = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminal_DAM, 0.0);

        // Shooting problem
        auto shooting_problem = boost::make_shared<crocoddyl::ShootingProblem>(x0_dummy, running_IAMs, terminal_IAM);
        ocp_ = boost::make_shared<mim_solvers::SolverCSQP>(shooting_problem);
        // ocp_ = boost::make_shared<mim_solvers::SolverSQP>(shooting_problem);
        // ocp_ = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);
        ocp_->set_termination_tolerance(config.solver_termination_tolerance);
        ocp_->set_max_qp_iters(config.max_qp_iter);
        ocp_->set_eps_abs(config.qp_termination_tol_abs);
        ocp_->set_eps_rel(config.qp_termination_tol_rel);
        ocp_->setCallbacks(false);
        
        // Callbacks from crocoddyl
        std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
        callbacks.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());


        // Callbacks from Mim Solvers
        std::cout << "costs term:   " << *terminalCostModel << std::endl;
        std::cout << "constraint term:   " << *terminalConstraintModelManager << std::endl;
        std::cout << "shooting pronlem:   " << *shooting_problem<< std::endl;

        std::cout << "ddp problem set up " << std::endl;
    }

    void CrocoddylReaching::change_obstacle_pose(const pin::SE3& pose, const std::string& geom_name)
    {
        collision_model_->geometryObjects[collision_model_->getGeometryId(geom_name)].placement = pose;
    }

    bool CrocoddylReaching::valid_pbe()
    {
        if (!posture_set_) return false;
        if (!(goal_translation_set_ || goal_placement_set_)) return false;
        return true;
    }

    bool CrocoddylReaching::solve(std::vector<Eigen::Matrix<double, -1, 1>> xs_init, std::vector<Eigen::Matrix<double, -1, 1>> us_init)
    {
        if (!valid_pbe()) return false;

        bool has_converged = ocp_->solve(xs_init, us_init, config_.nb_iterations_max, false);

        // generally does not converge within 1 iteration so pointless to return it
        return true;
    }

    Vector7d CrocoddylReaching::get_tau_ff() const
    {
        return ocp_->get_us()[0];
    }

    Eigen::MatrixXd CrocoddylReaching::get_ricatti_mat() const
    {
        return ocp_->get_K()[0];
    }

    std::pair<double,double> CrocoddylReaching::get_targets_weights(const double& time,const int& node_index){
            double cycle_start_time =double(int(time/targ_config_.cycle_duration) * targ_config_.cycle_duration);  // Date of cycle start in ms

            // Compute the absolute time of the shooting interval, modulo the cycle time,
            // so that 0<=time_a0<cycle_duration and 0<time_a1<=cycle_duration
            double time_a0 = time + node_index * config_.dt_ocp - cycle_start_time; // absolute data of the time of the start of the shooting interval
            double time_a1 = time + (node_index + 1) * config_.dt_ocp - cycle_start_time; // absolute data of the time of the start of the shooting interval
            if (time_a0 > targ_config_.cycle_duration){
                time_a0 -= targ_config_.cycle_duration;
                time_a1 -= targ_config_.cycle_duration;
            }
            double time_b0, time_b1; 
            // Compute the absolute time of the shooting interval for the second task, modulo the cycle time,
            // so that 0<=time_b0<cycle_duration and 0<time_b1<=cycle_duration and [time_a0,time_a1] is in antiphase with [time_b0,time_b1].
            if (time_a0 < targ_config_.cycle_duration_2){
                time_b0 = time_a0 + targ_config_.cycle_duration_2;
                if (time_a1 <= targ_config_.cycle_duration_2){
                    time_b1 = time_a1 + targ_config_.cycle_duration_2;
                }else{
                    time_b1 = targ_config_.cycle_duration;
                }
            }
            else{
                time_b0 = time_a0 - targ_config_.cycle_duration_2;
                time_b1 = time_a1 - targ_config_.cycle_duration_2;
            }

            if (time_a1 > targ_config_.cycle_duration){
                time_a1 =targ_config_.cycle_duration;
            }

            // Compute the weights for the first task, as \integral_time_a0^time_a1 weight(s) ds.
            double weight_a = targ_config_.max_w*std::exp(-targ_config_.w_slope*(targ_config_.cycle_duration-time_a0)/targ_config_.cycle_duration);

            // Compute the weights for the second task, as \integral_time_b0^time_b1 weight(s) ds.
            double weight_b = targ_config_.max_w*std::exp(-targ_config_.w_slope*(targ_config_.cycle_duration-time_b0)/targ_config_.cycle_duration);

            return std::make_pair(weight_a,weight_b);
        }

    std::pair<double,pin::SE3> CrocoddylReaching::get_weight_and_target(const double& time,const int& node_index){
        std::pair<double,double> weights = get_targets_weights(time, node_index);
        double weight_a = weights.first;
        double weight_b = weights.second;
        if (node_index == 0){
            if (weight_a > weight_b){
                targ_config_.weight_a_is_target= true;
            }else{
                targ_config_.weight_a_is_target= false;
            }
        }

        const int current_target_idx = int(std::fmod(time ,targ_config_.cycle_duration) * targ_config_.publish_frequency);
        int next_target_idx = (current_target_idx+1)%targ_config_.nb_target;
        pin::SE3 current_target = targ_config_.pose_targets[current_target_idx];
        pin::SE3 next_target = targ_config_.pose_targets[next_target_idx];
        pin::SE3 target;
        double weight;
        if (weight_a > weight_b){
            weight = weight_a;
        }else{
            weight = weight_b;
        }
        target.rotation() = current_target.rotation();
        if ((targ_config_.weight_a_is_target && weight_a > weight_b) || (!targ_config_.weight_a_is_target && weight_b > weight_a)){
            target.translation() = current_target.translation();
        }else{
            target.translation() = next_target.translation();
        }
        return std::make_pair(weight,target);
    }

    void CrocoddylReaching::set_ee_ref_translation_changing_weights(double time, bool is_active)
    {
        // Running
        for (size_t node_index = 0; node_index < config_.T; node_index++)
        {
            auto running_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ocp_->get_problem()->get_runningModels()[node_index]);
            auto running_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(running_IAM->get_differential());
            auto frame_res_running = boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(running_DAM->get_costs()->get_costs().at(cost_translation_name_)->cost->get_residual());
            std::pair<double,pin::SE3> weight_and_target =  get_weight_and_target(time, node_index);
            pin::SE3 target = weight_and_target.second;
            frame_res_running->set_reference(target.translation());
            if (!goal_translation_set_)
            {
                running_DAM->get_costs()->changeCostStatus(cost_translation_name_, is_active);
            }

            double weight = weight_and_target.first;
            running_DAM->get_costs()->get_costs().at(cost_translation_name_)->weight =weight;
        }

        // Terminal
        auto terminal_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ocp_->get_problem()->get_terminalModel());
        auto terminal_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(terminal_IAM->get_differential());
        auto frame_res_terminal = boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(terminal_DAM->get_costs()->get_costs().at(cost_translation_name_)->cost->get_residual());
        std::pair<double,pin::SE3> weight_and_target =  get_weight_and_target(time, config_.T);
        pin::SE3 target = weight_and_target.second;
        frame_res_terminal->set_reference(target.translation());
        if (!goal_translation_set_)
        {
            terminal_DAM->get_costs()->changeCostStatus(cost_translation_name_, is_active);

            // No need to activate again
            goal_translation_set_ = is_active;
        }


        double weight = weight_and_target.first;
        terminal_DAM->get_costs()->get_costs().at(cost_translation_name_)->weight =weight;
    }

    void CrocoddylReaching::set_ee_ref_translation_constant_weights(Eigen::Vector3d trans, bool is_active)
    {
        // Running
        for (size_t node_index = 0; node_index < config_.T; node_index++)
        {
            auto running_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ocp_->get_problem()->get_runningModels()[node_index]);
            auto running_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(running_IAM->get_differential());
            auto frame_res_running = boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(running_DAM->get_costs()->get_costs().at(cost_translation_name_)->cost->get_residual());
            frame_res_running->set_reference(trans);
            if (!goal_translation_set_)
            {
                running_DAM->get_costs()->changeCostStatus(cost_translation_name_, is_active);
            }
        }

        // Terminal
        auto terminal_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ocp_->get_problem()->get_terminalModel());
        auto terminal_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(terminal_IAM->get_differential());
        auto frame_res_terminal = boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(terminal_DAM->get_costs()->get_costs().at(cost_translation_name_)->cost->get_residual());
        frame_res_terminal->set_reference(trans);
        if (!goal_translation_set_)
        {
            terminal_DAM->get_costs()->changeCostStatus(cost_translation_name_, is_active);

            // No need to activate again
            goal_translation_set_ = is_active;
        }
    }

    void CrocoddylReaching::set_ee_ref_placement_changing_weights(double time, bool is_active, double uniform_weight_scaling)
    {
        // Running
        for (size_t node_index = 0; node_index < config_.T; node_index++)
        {
            auto running_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ocp_->get_problem()->get_runningModels()[node_index]);
            auto running_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(running_IAM->get_differential());
            
             //auto cost =  boost::static_pointer_cast<CostModelResidual>()
            std::pair<double,pin::SE3> weight_and_target =  get_weight_and_target(time, node_index);
            double weight = weight_and_target.first;
            running_DAM->get_costs()->get_costs().at(cost_placement_name_)->weight =weight;
            auto frame_res_running = boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement>(running_DAM->get_costs()->get_costs().at(cost_placement_name_)->cost->get_residual());
            pin::SE3 target = weight_and_target.second;
            frame_res_running->set_reference(target);

            if (!goal_placement_set_)
            {
                running_DAM->get_costs()->changeCostStatus(cost_placement_name_, is_active);
            }
        }

        // Terminal
        auto terminal_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ocp_->get_problem()->get_terminalModel());
        auto terminal_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(terminal_IAM->get_differential());
        auto frame_res_terminal = boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement>(terminal_DAM->get_costs()->get_costs().at(cost_placement_name_)->cost->get_residual());
        
        
        std::pair<double,pin::SE3> weight_and_target =  get_weight_and_target(time, config_.T);
        pin::SE3 target = weight_and_target.second;

        frame_res_terminal->set_reference(target);
        if (!goal_placement_set_)
        {
            terminal_DAM->get_costs()->changeCostStatus(cost_placement_name_, is_active);

            // No need to activate again
            goal_placement_set_ = is_active;
        }

        double weight = weight_and_target.first;
        terminal_DAM->get_costs()->get_costs().at(cost_placement_name_)->weight =weight;
    }

    void CrocoddylReaching::set_ee_ref_placement_constant_weights(pin::SE3 placement, bool is_active, double uniform_weight_scaling)
    {
        // Running
        for (size_t node_index = 0; node_index < config_.T; node_index++)
        {
            auto running_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ocp_->get_problem()->get_runningModels()[node_index]);
            auto running_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(running_IAM->get_differential());
            auto frame_res_running = boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement>(running_DAM->get_costs()->get_costs().at(cost_placement_name_)->cost->get_residual());
            frame_res_running->set_reference(placement);
            if (!goal_placement_set_)
            {
                running_DAM->get_costs()->changeCostStatus(cost_placement_name_, is_active);
            }

            running_DAM->get_costs()->get_costs().at(cost_placement_name_)->weight = uniform_weight_scaling * config_.w_frame_running;
        }

        // Terminal
        auto terminal_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ocp_->get_problem()->get_terminalModel());
        auto terminal_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(terminal_IAM->get_differential());
        auto frame_res_terminal = boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement>(terminal_DAM->get_costs()->get_costs().at(cost_placement_name_)->cost->get_residual());
        frame_res_terminal->set_reference(placement);
        if (!goal_placement_set_)
        {
            terminal_DAM->get_costs()->changeCostStatus(cost_placement_name_, is_active);

            // No need to activate again
            goal_placement_set_ = is_active;
        }

        terminal_DAM->get_costs()->get_costs().at(cost_placement_name_)->weight = uniform_weight_scaling * config_.w_frame_running;
    }


    void CrocoddylReaching::set_posture_ref(Eigen::VectorXd x0)
    {
        assert(x0.size() == ocp_->get_problem()->get_nx());

        // Running
        for (size_t node_index = 0; node_index < config_.T; node_index++)
        {
            auto running_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ocp_->get_problem()->get_runningModels()[node_index]);
            auto running_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(running_IAM->get_differential());
            auto posture_res_running = boost::static_pointer_cast<crocoddyl::ResidualModelState>(running_DAM->get_costs()->get_costs().at(cost_state_reg_name_)->cost->get_residual());
            posture_res_running->set_reference(x0);
            if (!posture_set_)
            {
                running_DAM->get_costs()->changeCostStatus(cost_state_reg_name_, true);
            }
        }

        // Terminal
        auto terminal_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ocp_->get_problem()->get_terminalModel());
        auto terminal_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(terminal_IAM->get_differential());
        auto posture_res_terminal = boost::static_pointer_cast<crocoddyl::ResidualModelState>(terminal_DAM->get_costs()->get_costs().at(cost_state_reg_name_)->cost->get_residual());
        posture_res_terminal->set_reference(x0);
        if (!posture_set_)
        {
            terminal_DAM->get_costs()->changeCostStatus(cost_state_reg_name_, true);

            // No need to activate again
            posture_set_ = true;
        }
    }

} // namespace panda_torque_mpc
