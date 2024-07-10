#include <pinocchio/fwd.hpp>
#include <boost/python.hpp>
#include <panda_torque_mpc/crocoddyl_reaching.h>


// include fwd first
#include <eigenpy/eigenpy.hpp>

namespace panda_torque_mpc {
    void exposeCrocoddylReaching();
    void exposeReduceCollisionModel();
}  // namespace python