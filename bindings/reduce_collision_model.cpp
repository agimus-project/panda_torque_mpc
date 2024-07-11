#include <pinocchio/multibody/fwd.hpp>
#include <boost/python.hpp>

#include "panda_torque_mpc/reduce_collision_model.h"
#include "panda_torque_mpc/python.h"

namespace panda_torque_mpc
{
namespace pin = pinocchio;
namespace bp = boost::python;

void exposeReduceCollisionModel()
{
  bp::register_ptr_to_python<boost::shared_ptr<pin::GeometryModel> >();
  bp::def("reduce_capsules_robot", &reduce_capsules_robot);
}

}  // namespace panda_torque_mpc
