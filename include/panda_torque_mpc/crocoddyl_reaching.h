#pragma once

#include <vector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>

#include "panda_torque_mpc/common.h"

namespace pin = pinocchio;

namespace panda_torque_mpc
{
    struct CrocoddylConfig
    {
        size_t T; // nb of nodes - terminal one
        double dt_ocp;
        size_t nb_iterations_max;

        std::string ee_frame_name;
        bool reference_is_placement = false;


        // Task weights
        double w_frame_running = 10.0;
        double w_frame_terminal = 1000.0;

        double w_x_reg_running = 1.0;
        double w_x_reg_terminal = 1.0;
        Eigen::Matrix<double, 7, 1> diag_q_reg_running = Eigen::Matrix<double, 7, 1>::Ones();
        Eigen::Matrix<double, 7, 1> diag_v_reg_running = Eigen::Matrix<double, 7, 1>::Ones();

        double w_u_reg_running = 0.01;
        Eigen::Matrix<double, 7, 1> diag_u_reg_running = Eigen::Matrix<double, 7, 1>::Ones();

        Eigen::Matrix<double, 7, 1> armature;
    };

    class CrocoddylReaching
    {
    public:
        CrocoddylReaching()
        {
            // dummy constructor necessary to use this class as a member variable directly
        }

        CrocoddylReaching(pin::Model _model_pin, CrocoddylConfig _config);

        void set_ee_ref_translation(Eigen::Vector3d trans, bool is_active=true);
        void set_ee_ref_placement(pin::SE3 placement, bool is_active=true);

        void set_posture_ref(Eigen::VectorXd x0);

        boost::shared_ptr<crocoddyl::SolverFDDP> ddp_;
        CrocoddylConfig config_;

        std::string goal_cost_translation_name_;
        std::string goal_cost_placement_name_;
        std::string state_reg_cost_name_;
        std::string ctrl_reg_cost_name_;
        
        // safe guards
        bool goal_translation_set_;
        bool goal_placement_set_;
        bool posture_set_;


    bool valid_pbe();
    bool solve(std::vector<Eigen::Matrix<double, -1, 1>> xs_init, std::vector<Eigen::Matrix<double, -1, 1>> us_init);
    Vector7d get_tau_ff() const;
    Eigen::MatrixXd get_ricatti_mat() const;

    };

} // namespace panda_torque_mpc