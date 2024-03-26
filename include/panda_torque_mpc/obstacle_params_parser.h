#pragma once

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <memory>
#include <vector>
#include <iostream>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <ros/ros.h>

namespace panda_torque_mpc {

class ObstacleParamsParser {
public:
  ObstacleParamsParser(
      const boost::shared_ptr<ros::NodeHandle> &pnh,
      const boost::shared_ptr<pinocchio::GeometryModel> &collision_model);
  ~ObstacleParamsParser();

  /**
   * @brief The collisions from the YAML file to the geometry model.
  */

  void addCollisions();

private:
  /**
    * @brief Add a collision pair between two objects.
    *
    * @param name_object1 The name of the first object in the collision pair.
    * @param name_object2 The name of the second object in the collision
    * pair.
    */
  void addCollisionPair(const std::string &name_object1,
                      const std::string &name_object2);

  const boost::shared_ptr<ros::NodeHandle> pnh_;
  const boost::shared_ptr<pinocchio::GeometryModel> &collision_model_;
};

} // namespace panda_torque_mpc
