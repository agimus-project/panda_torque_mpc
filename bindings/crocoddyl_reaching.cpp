

#include <pinocchio/multibody/fwd.hpp>
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <vector>

#include "panda_torque_mpc/crocoddyl_reaching.h"
#include "panda_torque_mpc/python.h"

namespace panda_torque_mpc
{
   
    namespace pin = pinocchio;
    namespace bp = boost::python;
    
   static boost::shared_ptr<CrocoddylReaching> constructor(pin::Model _model_pin, 
            const boost::shared_ptr<pin::GeometryModel>& _collision_model,bp::dict mpc_params) {

   CrocoddylConfig croco_conf;
   croco_conf.T = bp::extract<size_t>(mpc_params["nb_shooting_nodes"]);
   croco_conf.dt_ocp = bp::extract<double>(mpc_params["dt_ocp"]);
   croco_conf.solver_termination_tolerance = bp::extract<double>(mpc_params["solver_termination_tolerance"]);
   croco_conf.qp_termination_tol_abs = bp::extract<double>(mpc_params["qp_termination_tol_abs"]);
   croco_conf.qp_termination_tol_rel = bp::extract<double>(mpc_params["qp_termination_tol_rel"]);
   croco_conf.nb_iterations_max = bp::extract<size_t>(mpc_params["nb_iterations_max"]);
   croco_conf.max_qp_iter = bp::extract<size_t>(mpc_params["max_qp_iter"]);
   croco_conf.ee_frame_name = bp::extract<std::string>(mpc_params["ee_frame_name"]);
   croco_conf.reference_is_placement = bp::extract<bool>(mpc_params["reference_is_placement"]);
   croco_conf.w_frame_running = bp::extract<double>(mpc_params["w_frame_running"]);
   croco_conf.w_frame_terminal = bp::extract<double>(mpc_params["w_frame_terminal"]);
   croco_conf.w_frame_vel_running = bp::extract<double>(mpc_params["w_frame_vel_running"]);
   croco_conf.w_frame_vel_terminal = bp::extract<double>(mpc_params["w_frame_vel_terminal"]);
   std::vector<double> diag_frame_vel =  bp::extract<std::vector<double>>(mpc_params["diag_frame_vel"]);
   croco_conf.diag_frame_vel = Eigen::Map<Eigen::Matrix<double, 6, 1>>(diag_frame_vel.data());
   croco_conf.w_x_reg_running = bp::extract<double>(mpc_params["w_x_reg_running"]);
   croco_conf.w_x_reg_terminal = bp::extract<double>(mpc_params["w_x_reg_terminal"]);
   std::vector<double> diag_q_reg_running =  bp::extract<std::vector<double>>(mpc_params["diag_q_reg_running"]);
   croco_conf.diag_q_reg_running = Eigen::Map<Eigen::Matrix<double, 7, 1>>(diag_q_reg_running.data());
   std::vector<double> diag_v_reg_running =  bp::extract<std::vector<double>>(mpc_params["diag_v_reg_running"]);
   croco_conf.diag_v_reg_running = Eigen::Map<Eigen::Matrix<double, 7, 1>>(diag_v_reg_running.data());
   croco_conf.w_u_reg_running = bp::extract<double>(mpc_params["w_u_reg_running"]);
   std::vector<double> diag_u_reg_running =  bp::extract<std::vector<double>>(mpc_params["diag_u_reg_running"]);
   croco_conf.diag_u_reg_running = Eigen::Map<Eigen::Matrix<double, 7, 1>>(diag_u_reg_running.data());
   std::vector<double> armature =  bp::extract<std::vector<double>>(mpc_params["armature"]);
   croco_conf.armature = Eigen::Map<Eigen::Matrix<double, 7, 1>>(armature.data());

   TargetsConfig targ_config_;
   std::vector<double> pose_target1 =  bp::extract<std::vector<double>>(mpc_params["pose_target1"]);
   std::vector<double> pose_target2 =  bp::extract<std::vector<double>>(mpc_params["pose_target2"]);
   targ_config_.pose_targets = {panda_torque_mpc::XYZQUATToSE3(pose_target1),panda_torque_mpc::XYZQUATToSE3(pose_target2)};
   targ_config_.publish_frequency = bp::extract<double>(mpc_params["publish_frequency"]);
   targ_config_.nb_target = targ_config_.pose_targets.size();
   targ_config_.cycle_nb_nodes = targ_config_.cycle_duration / croco_conf.dt_ocp;
   targ_config_.cycle_duration = targ_config_.nb_target/targ_config_.publish_frequency;
   targ_config_.cycle_duration_2 = targ_config_.cycle_duration /2;
   targ_config_.w_slope = bp::extract<double>(mpc_params["w_slope"]);
   targ_config_.max_w = bp::extract<double>(mpc_params["max_w"]);
   targ_config_.w_cut = bp::extract<double>(mpc_params["w_cut"]);
   targ_config_.weight_a_is_target = true;
   
   return boost::shared_ptr<CrocoddylReaching>(new CrocoddylReaching(_model_pin, _collision_model,croco_conf,targ_config_));
   }

    void exposeCrocoddylReaching() {
      
      bp::register_ptr_to_python<boost::shared_ptr<CrocoddylReaching> >(); 
      bp::class_<CrocoddylReaching>("CrocoddylReaching", bp::no_init)
        .def("__init__", bp::make_constructor(&constructor))
        .def<std::pair<double,double> (CrocoddylReaching::*)(const double& ,const int& ) >("get_targets_weights", &CrocoddylReaching::get_targets_weights,
           bp::args("self", "time", "node_index"),
           "Return weights of the two targets.\n\n"
           ":param time: time\n"
           ":param node_index: node index")
        .def<std::pair<double,pin::SE3> (CrocoddylReaching::*)(const double& ,const int& ) >("get_weight_and_target", &CrocoddylReaching::get_weight_and_target,
           bp::args("self", "time", "node_index"),
           "Return weights and target.\n\n"
           ":param time: time\n"
           ":param node_index: node index")
        .def("set_ee_ref_placement", &CrocoddylReaching::set_ee_ref_placement,
           bp::args("self", "placement", "time","is_active","uniform_weight_scaling"),
           "update ocp.\n\n"
           ":param placement: placement\n"
           ":param time: time\n"
           ":param is_active: is_active\n"
           ":param uniform_weight_scaling: uniform_weight_scaling")
        /*.def<void (CrocoddylReaching::*)(Eigen::Matrix<double, -1, 1> ,std::vector<Eigen::Matrix<double, -1, 1>> ,std::vector<Eigen::Matrix<double, -1, 1>> ,double)>("solving", &CrocoddylReaching::solving,
           bp::args("self", "x0", "xs_init","us_init","time"),
           "Solve the problem.\n\n")*/
        .add_property("solver", &CrocoddylReaching::get_solver);
    }

} // namespace panda_torque_mpc
