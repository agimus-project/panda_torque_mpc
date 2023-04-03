#pragma once

#include <vector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>


#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>



namespace pin = pinocchio;

struct CrocoddylConfig
{
    size_t T; // nb of nodes - terminal one
    double dt_ocp;
    size_t nb_iterations_max;

    std::string ee_frame_name;

    Eigen::Matrix<double, 7, 1> armature;

    // Task weights
    double w_frame_running = 100.0;
    double w_frame_terminal = 1000.0;

    double w_x_reg_running = 1.0;
    double w_x_reg_terminal = 0.01;
    double scale_q_vs_v_reg = 0.1;

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

    CrocoddylReaching(pin::Model _model_pin, CrocoddylConfig _config);

    void set_ee_ref(Eigen::Vector3d trans);

    void set_posture_ref(Eigen::VectorXd x0);

    boost::shared_ptr<crocoddyl::ActionModelAbstract> terminal_IAM_;
    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_IAMs_;

    boost::shared_ptr<crocoddyl::SolverFDDP> ddp_;
    CrocoddylConfig config_;

    std::string goal_cost_name_;
    // safe guards
    bool goal_translation_set_;
    bool posture_set_;
};