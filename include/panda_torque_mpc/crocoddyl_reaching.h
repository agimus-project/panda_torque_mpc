#pragma once

#include <vector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>

#include <colmpc/fwd.hpp>
#include <colmpc/residual-distance-collision.hpp>

#include <mim_solvers/csqp.hpp>
#include <mim_solvers/sqp.hpp>

#include "panda_torque_mpc/common.h"

namespace pin = pinocchio;

namespace panda_torque_mpc
{
    struct TargetsConfig
    {
        double publish_frequency = 0.5;
        int nb_target = 2;
        int cycle_nb_nodes;
        double cycle_duration;
        double cycle_duration_2;
        double w_slope;
        double max_w;
        double w_cut;
        bool weight_a_is_target = true;
        std::vector<pin::SE3> pose_targets;
    };
    struct CrocoddylConfig
    {
        size_t T; // nb of nodes - terminal one
        double dt_ocp;
        double solver_termination_tolerance;
        double qp_termination_tol_abs;
        double qp_termination_tol_rel;
        size_t nb_iterations_max;
        size_t max_qp_iter;

        std::string ee_frame_name;
        bool reference_is_placement = false;


        // Task weights
        double w_frame_running = 10.0;
        double w_frame_terminal = 1000.0;
        
        double w_frame_vel_running = 10.0;
        double w_frame_vel_terminal = 1000.0;
        Eigen::Matrix<double, 6, 1> diag_frame_vel = Eigen::Matrix<double, 6, 1>::Ones();

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

        CrocoddylReaching(pin::Model _model_pin, const boost::shared_ptr<pin::GeometryModel>& _collision_model ,CrocoddylConfig _config, TargetsConfig _targ_config);
        // CrocoddylReaching(pin::Model _model_pin, CrocoddylConfig _config);

        // Return weights for current and next target
        std::pair<double,double> get_targets_weights(const double& time,const int& node_index);

        // Return current target and his weight
        std::pair<double,pin::SE3> get_weight_and_target(const double& time,const int& node_index);
        
        void set_ee_ref_translation(Eigen::Vector3d trans, double time, bool is_active=true);
        /**
         * placement: pin::SE3, reference placement, constant for the whole horizon
         * is_active: bool, activate/deactivate the costs over the whole horizon
         * uniform_weight_scaling: double, constant for the whole horizon
        */
        void set_ee_ref_placement(pin::SE3 placement, double time, bool is_active=true, double uniform_weight_scaling=1.0);

        void set_posture_ref(Eigen::VectorXd x0);

        void set_with_collision(const bool enable) {config_.with_collisions = enable;}

        // boost::shared_ptr<crocoddyl::SolverFDDP> ocp_;
        // boost::shared_ptr<mim_solvers::SolverSQP> ocp_;
        boost::shared_ptr<mim_solvers::SolverCSQP> ocp_;

        TicTac simulation_time;
        CrocoddylConfig config_;
        TargetsConfig targ_config_;

        std::string cost_translation_name_;
        std::string cost_placement_name_;
        std::string cost_velocity_name_;
        std::string cost_state_reg_name_;
        std::string cost_ctrl_reg_name_;
        
        
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