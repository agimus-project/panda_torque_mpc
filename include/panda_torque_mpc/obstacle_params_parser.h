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

class ObstacleParamsParser {
public:
  ObstacleParamsParser(
      const std::shared_ptr<ros::NodeHandle> &nh,
      const boost::shared_ptr<pinocchio::Model> &pin_model_,
      const boost::shared_ptr<pinocchio::GeometryModel> &collision_model_);
  ~ObstacleParamsParser();

public:

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
   * @brief Add a sphere to the geometry model.
   *
   * @param name The name of the obstacle.
   * @param radius The radius of the sphere.
   * @param pose The pose of the obstacle specified as [tx, ty, tz, qx, qy, qz,
   * qw].
   */
  void addSphere(const std::string &name, const double &radius,
                 const Eigen::VectorXd &pose);
  
  /**
   * @brief Add a box to the geometry model.
   *
   * @param name The name of the obstacle.
   * @param dim The dimensions of the box.
   * @param pose The pose of the obstacle specified as [tx, ty, tz, qx, qy, qz,
   * qw].
   */
  void addBox(const std::string &name, const double Eigen::VectorXd &dim
                 const Eigen::VectorXd &pose)
 
  /**
   * @brief Add a capsule to the geometry model.
   *
   * @param name The name of the obstacle.
   * @param dim The dimensions of the capsule (radius then halfLength).
   * @param pose The pose of the obstacle specified as [tx, ty, tz, qx, qy, qz,
   * qw].
   */
  void addCapsule(const std::string &name, const double Eigen::VectorXd &dim
                 const Eigen::VectorXd &pose)
 
 
  /**
   * @brief Add a collision pair between two objects.
   *
   * @param name_object1 The name of the first object in the collision pair.
   * @param name_object2 The name of the second object in the collision pair.
   */
  void addCollisionPair(const std::string &name_object1,
                        const std::string &name_object2);

  std::shared_ptr<ros::NodeHandle> nh_;
  const boost::shared_ptr<pinocchio::Model> pin_model_;
  const boost::shared_ptr<pinocchio::GeometryModel> &collision_model_;
};

} // namespace panda_torque_mpc
