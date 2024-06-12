#include "panda_torque_mpc/python.h"
#include <eigenpy/eigenpy.hpp>

BOOST_PYTHON_MODULE(panda_torque_mpc_pywrap) {
  namespace bp = boost::python;

  bp::import("pinocchio");
  bp::import("crocoddyl");
  // Enabling eigenpy support, i.e. numpy/eigen compatibility.
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::VectorXi);
  ENABLE_SPECIFIC_MATRIX_TYPE(panda_torque_mpc::Vector7d);
  panda_torque_mpc::exposeCrocoddylReaching();
  
}