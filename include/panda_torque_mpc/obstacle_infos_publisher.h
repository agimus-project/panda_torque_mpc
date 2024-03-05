#pragma once

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <memory>
#include <vector>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ros/ros.h>

namespace panda_torque_mpc {

class ObstacleInfosPublisher {
public:
  ObstacleInfosPublisher(
      const std::shared_ptr<ros::NodeHandle> &ph,
      const boost::shared_ptr<pinocchio::Model> &pin_model_,
      const boost::shared_ptr<pinocchio::GeometryModel> &collision_model_);
  ~ObstacleInfosPublisher();

public:
  void addCollisions();

private:
  /**
   * @brief Add an obstacle to the geometry model.
   *
   * @param name The name of the obstacle.
   * @param type The type of the obstacle (e.g., "sphere").
   * @param dim The dimensions of the obstacle (e.g., radius for a sphere).
   * @param pose The pose of the obstacle specified as [tx, ty, tz, qx, qy, qz,
   * qw].
   */
  void addObstacle(const std::string &name, const std::string &type,
                   const Eigen::VectorXd &dim, const Eigen::VectorXd &pose);

  /**
   * @brief Add a collision pair between two objects.
   *
   * @param name_object1 The name of the first object in the collision pair.
   * @param name_object2 The name of the second object in the collision pair.
   */
  void addCollisionPair(const std::string &name_object1,
                        const std::string &name_object2);

  std::shared_ptr<ros::NodeHandle> ph_;
  const boost::shared_ptr<pinocchio::GeometryModel> &collision_model_;
};

} // namespace panda_torque_mpc
