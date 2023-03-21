#include <iostream>
#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/frame.hpp>

#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/solver-base.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/core/activations/quadratic.hpp>
// #include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
// #include <crocoddyl/core/activations/weighted-quadratic-barrier.hpp>
// #include <crocoddyl/core/activations/quadratic-flat-log.hpp>

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/solvers/ddp.hpp>
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/residuals/control.hpp>

#include <crocoddyl/multibody/costs/control-gravity.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/frames.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
// #include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
// #include <crocoddyl/multibody/residuals/contact-force.hpp>

namespace pin = pinocchio;

struct CrocoddylConfig
{
    unsigned int T; // nb of nodes - terminal one
    double dt_ocp;

    Eigen::Matrix<double, 14, 1> x0;
    std::string ee_frame_name;

    Eigen::Matrix<double, 7, 1> armature;

    // Task weights
    double w_frame_running = 100.0;
    double w_frame_terminal = 1000.0;

    double w_x_reg_running = 1.0;
    double w_x_reg_terminal = 0.01;
    double scale_q_reg = 0.1;

    double w_u_reg_running = 0.01;
    Eigen::Matrix<double, 7, 1> diag_u_reg_running = Eigen::Matrix<double, 7, 1>::Ones();
    
};

class CrocoddylReaching
{
public:

    CrocoddylReaching()
    {
        // dummy constructor necessary to use this class as a member variable directly
    }


    CrocoddylReaching(pin::Model _model_pin, CrocoddylConfig _config)
    {
        config_ = _config;
        auto end_effector_frame_id = _model_pin.getFrameId(_config.ee_frame_name);

        std::cout << "Creating state, actuation and IAMs... " << std::endl;
        boost::shared_ptr<crocoddyl::StateMultibody> state = boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(_model_pin));
        boost::shared_ptr<crocoddyl::ActuationModelFull> actuation = boost::make_shared<crocoddyl::ActuationModelFull>(state);


        Eigen::Matrix<double, 7, 1> diag_q_reg_running = _config.scale_q_reg * Eigen::Matrix<double, 7, 1>::Ones();
        Eigen::Matrix<double, 7, 1> diag_v_reg_running = Eigen::Matrix<double, 7, 1>::Ones();
        Eigen::Matrix<double, 14, 1> diag_x_reg_running; diag_x_reg_running << diag_q_reg_running, diag_v_reg_running;

        Eigen::Matrix<double, 14, 1> diag_x_reg_terminal = diag_x_reg_running;

        // !! translation reference should be set with set_goal_translation
        Eigen::Vector3d dummy_translation_reference;

        // Frame translation
        boost::shared_ptr<crocoddyl::CostModelAbstract> frame_goal_cost =
            boost::make_shared<crocoddyl::CostModelResidual>(state,
                                                             boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state, end_effector_frame_id, dummy_translation_reference, actuation->get_nu()));

        // Frame velocity cost
        // boost::shared_ptr<crocoddyl::CostModelAbstract> frame_velocity_cost =
        //     boost::make_shared<crocoddyl::CostModelResidual>(state,
        //                                                      boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(state, end_effector_frame_id, pinocchio::Motion::Zero(), pinocchio::LOCAL_WORLD_ALIGNED, actuation->get_nu()));

        running_models_ = std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>(_config.T);

        goal_cost_name_ = "translation_cost";

        for (int i = 0; i < _config.T; i++)
        {
            // State reg
            boost::shared_ptr<crocoddyl::CostModelAbstract> state_reg_cost =
                boost::make_shared<crocoddyl::CostModelResidual>(state,
                                                                 boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(diag_x_reg_running),
                                                                 boost::make_shared<crocoddyl::ResidualModelState>(state, _config.x0, actuation->get_nu()));

            // Ctrl reg
            boost::shared_ptr<crocoddyl::CostModelAbstract> ctrl_reg_cost =
                boost::make_shared<crocoddyl::CostModelResidual>(state,
                                                                 boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(_config.diag_u_reg_running),
                                                                 boost::make_shared<crocoddyl::ResidualModelControlGrav>(state, actuation->get_nu()));

            boost::shared_ptr<crocoddyl::CostModelSum> runningCostModel = boost::make_shared<crocoddyl::CostModelSum>(state);
            runningCostModel.get()->addCost("state_reg", state_reg_cost, _config.w_x_reg_running);
            runningCostModel.get()->addCost("ctrl_reg", ctrl_reg_cost, _config.w_u_reg_running);
            runningCostModel.get()->addCost(goal_cost_name_, frame_goal_cost, _config.w_frame_running); // TODO: weight schedule

            boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> runningDAM =
                boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, runningCostModel);
            runningDAM->set_armature(_config.armature);
            // Deactivate goal cost by default until a proper reference is set
            runningDAM->get_costs()->changeCostStatus(goal_cost_name_, false);

            running_models_[i] = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(runningDAM, _config.dt_ocp);
        }

        boost::shared_ptr<crocoddyl::CostModelSum> terminalCostModel = boost::make_shared<crocoddyl::CostModelSum>(state);
        // State reg
        boost::shared_ptr<crocoddyl::CostModelAbstract> state_reg_cost =
            boost::make_shared<crocoddyl::CostModelResidual>(state,
                                                             boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(diag_x_reg_terminal),
                                                             boost::make_shared<crocoddyl::ResidualModelState>(state, _config.x0, actuation->get_nu()));

        terminalCostModel.get()->addCost("state_reg", state_reg_cost, _config.w_x_reg_terminal);
        terminalCostModel.get()->addCost(goal_cost_name_, frame_goal_cost, _config.w_frame_terminal);
        // terminalCostModel.get()->addCost("terminal_vel", frame_velocity_cost, _config.w_frame_vel_terminal);

        boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> terminalDAM =
            boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, terminalCostModel);
        terminalDAM->set_armature(_config.armature);
        terminalDAM->get_costs()->changeCostStatus(goal_cost_name_, false);

        boost::shared_ptr<crocoddyl::ActionModelAbstract> terminal_model = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminalDAM, 0.0);

        // Shooting problem
        boost::shared_ptr<crocoddyl::ShootingProblem> shooting_problem =
            boost::make_shared<crocoddyl::ShootingProblem>(_config.x0, running_models_, terminal_model);
        ddp_ = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);

        // Callbacks
        std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
        callbacks.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());

        std::cout << "ddp problem set up " << std::endl;
    }

    void set_goal_translation(Eigen::Vector3d trans)
    {
        // Running
        for (size_t node_index = 0; node_index < config_.T; node_index++)
        {
            auto runningIAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ddp_->get_problem()->get_runningModels()[node_index]);
            auto runningDAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(runningIAM->get_differential());
            auto frame_res_running = boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(runningDAM->get_costs()->get_costs().at(goal_cost_name_)->cost->get_residual());
            frame_res_running->set_reference(trans);
            if (!goal_translation_set_)
            {
                runningDAM->get_costs()->changeCostStatus(goal_cost_name_, true);
            }
        }

        // Terminal
        auto terminalIAM = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(ddp_->get_problem()->get_terminalModel());
        auto terminalDAM = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(terminalIAM->get_differential());
        auto frame_res_terminal = boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(terminalDAM->get_costs()->get_costs().at(goal_cost_name_)->cost->get_residual());
        frame_res_terminal->set_reference(trans);
        terminalDAM->get_costs()->changeCostStatus(goal_cost_name_, false);
        if (!goal_translation_set_)
        {
            terminalDAM->get_costs()->changeCostStatus(goal_cost_name_, true);

            // No need to activate again
            goal_translation_set_ = true;
        }
    }

    boost::shared_ptr<crocoddyl::ActionModelAbstract> terminal_model_;
    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_models_;

    boost::shared_ptr<crocoddyl::SolverFDDP> ddp_;
    CrocoddylConfig config_;

    std::string goal_cost_name_;
    // safe guard
    bool goal_translation_set_;
};