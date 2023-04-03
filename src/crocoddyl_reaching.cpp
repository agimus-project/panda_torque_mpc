

#include <iostream>
#include <algorithm>
#include <Eigen/Dense>

#include "panda_torque_mpc/crocoddyl_reaching.h"

// #include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/algorithm/joint-configuration.hpp>

// #include <crocoddyl/core/fwd.hpp>
// #include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/activations/quadratic.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>

#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/frames.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/costs/control-gravity.hpp>



CrocoddylReaching::CrocoddylReaching(pin::Model _model_pin, CrocoddylConfig _config)
{
    config_ = _config;
    auto end_effector_frame_id = _model_pin.getFrameId(_config.ee_frame_name);

    std::cout << "Creating state, actuation and IAMs... " << std::endl;
    auto state = boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(_model_pin));
    auto actuation = boost::make_shared<crocoddyl::ActuationModelFull>(state);

    Eigen::Matrix<double, 7, 1> diag_q_reg_running = _config.scale_q_vs_v_reg * Eigen::Matrix<double, 7, 1>::Ones();
    Eigen::Matrix<double, 7, 1> diag_v_reg_running = Eigen::Matrix<double, 7, 1>::Ones();
    Eigen::Matrix<double, 14, 1> diag_x_reg_running;
    diag_x_reg_running << diag_q_reg_running, diag_v_reg_running;

    Eigen::Matrix<double, 14, 1> diag_x_reg_terminal = diag_x_reg_running;

    // !! translation reference should be set with set_ee_ref
    goal_translation_set_ = false;
    posture_set_ = false;
    Eigen::Vector3d dummy_translation_reference = Eigen::Vector3d::Zero();
    // !! posture reference should be set set_posture_ref
    Eigen::Matrix<double, 14, 1> x0_dummy = Eigen::Matrix<double, 14, 1>::Zero();

    // Frame translation
    auto frame_goal_cost = boost::make_shared<crocoddyl::CostModelResidual>(
        state,
        boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state, end_effector_frame_id, dummy_translation_reference, actuation->get_nu()));

    running_IAMs_ = std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>(_config.T);

    goal_cost_name_ = "translation_cost";

    for (int i = 0; i < _config.T; i++)
    {
        // State reg
        auto state_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(
            state,
            boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(diag_x_reg_running),
            boost::make_shared<crocoddyl::ResidualModelState>(state, x0_dummy, actuation->get_nu()));

        // Ctrl reg
        auto ctrl_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(
            state,
            boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(_config.diag_u_reg_running),
            boost::make_shared<crocoddyl::ResidualModelControlGrav>(state, actuation->get_nu()));

        auto runningCostModel = boost::make_shared<crocoddyl::CostModelSum>(state);
        runningCostModel.get()->addCost("state_reg", state_reg_cost, _config.w_x_reg_running);
        runningCostModel.get()->addCost("ctrl_reg", ctrl_reg_cost, _config.w_u_reg_running);
        runningCostModel.get()->addCost(goal_cost_name_, frame_goal_cost, _config.w_frame_running); // TODO: weight schedule

        auto running_DAM = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, runningCostModel);
        running_DAM->set_armature(_config.armature);
        // Deactivate goal cost by default until a proper reference is set
        running_DAM->get_costs()->changeCostStatus(goal_cost_name_, false);

        running_IAMs_[i] = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(running_DAM, _config.dt_ocp);
    }

    auto terminalCostModel = boost::make_shared<crocoddyl::CostModelSum>(state);
    // State reg
    auto state_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(
        state,
        boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(diag_x_reg_terminal),
        boost::make_shared<crocoddyl::ResidualModelState>(state, x0_dummy, actuation->get_nu()));

    terminalCostModel.get()->addCost("state_reg", state_reg_cost, _config.w_x_reg_terminal);
    terminalCostModel.get()->addCost(goal_cost_name_, frame_goal_cost, _config.w_frame_terminal);

    auto terminal_DAM = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, terminalCostModel);
    terminal_DAM->set_armature(_config.armature);
    terminal_DAM->get_costs()->changeCostStatus(goal_cost_name_, false);

    auto terminal_IAM = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminal_DAM, 0.0);

    // Shooting problem
    auto shooting_problem = boost::make_shared<crocoddyl::ShootingProblem>(x0_dummy, running_IAMs_, terminal_IAM);
    ddp_ = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);

    // Callbacks
    std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
    callbacks.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());

    std::cout << "ddp problem set up " << std::endl;
}

void CrocoddylReaching::set_ee_ref(Eigen::Vector3d trans)
{
    // Running
    for (size_t node_index = 0; node_index < config_.T; node_index++)
    {
        auto running_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ddp_->get_problem()->get_runningModels()[node_index]);
        auto running_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(running_IAM->get_differential());
        auto frame_res_running = boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(running_DAM->get_costs()->get_costs().at(goal_cost_name_)->cost->get_residual());
        frame_res_running->set_reference(trans);
        if (!goal_translation_set_)
        {
            running_DAM->get_costs()->changeCostStatus(goal_cost_name_, true);
        }
    }

    // Terminal
    auto terminal_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ddp_->get_problem()->get_terminalModel());
    auto terminal_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(terminal_IAM->get_differential());
    auto frame_res_terminal = boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(terminal_DAM->get_costs()->get_costs().at(goal_cost_name_)->cost->get_residual());
    frame_res_terminal->set_reference(trans);
    if (!goal_translation_set_)
    {
        terminal_DAM->get_costs()->changeCostStatus(goal_cost_name_, true);

        // No need to activate again
        goal_translation_set_ = true;
    }
}

void CrocoddylReaching::set_posture_ref(Eigen::VectorXd x0)
{
    assert(x0.size() == ddp_->get_problem()->get_nx());

    // Running
    for (size_t node_index = 0; node_index < config_.T; node_index++)
    {
        auto running_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ddp_->get_problem()->get_runningModels()[node_index]);
        auto running_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(running_IAM->get_differential());
        auto posture_res_running = boost::static_pointer_cast<crocoddyl::ResidualModelState>(running_DAM->get_costs()->get_costs().at("state_reg")->cost->get_residual());
        posture_res_running->set_reference(x0);
        if (!posture_set_)
        {
            running_DAM->get_costs()->changeCostStatus("state_reg", true);
        }
    }

    // Terminal
    auto terminal_IAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ddp_->get_problem()->get_terminalModel());
    auto terminal_DAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(terminal_IAM->get_differential());
    auto posture_res_terminal = boost::static_pointer_cast<crocoddyl::ResidualModelState>(terminal_DAM->get_costs()->get_costs().at("state_reg")->cost->get_residual());
    posture_res_terminal->set_reference(x0);
    if (!posture_set_)
    {
        terminal_DAM->get_costs()->changeCostStatus("state_reg", true);

        // No need to activate again
        posture_set_ = true;
    }
}
