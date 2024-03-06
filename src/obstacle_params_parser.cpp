#pragma once

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <memory>
#include <vector>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace panda_torque_mpc {

class ObstacleParamsParser {
  ObstacleParamsParser(
      const std::shared_ptr<ros::NodeHandle> &nh,
      const boost::shared_ptr<pinocchio::Model> &pin_model,
      const boost::shared_ptr<pinocchio::GeometryModel> &collision_model)
      : nh_(nh), pin_model_(pin_model), collision_model_(collision_model){};

  ~ObstacleParamsParser() {};

void ObstacleParamsParser::addCollisions();
{
  // First adding the obstacles to the geometry model
 int obs_idx = 0;
 while (nh_->hasParam("obstacle_" + std::to_string(obs_idx))) {
  std::string type;
  nh_->getParam(obstacle_name + "/type", type);
  switch (type) {
    case "sphere":
      double radius;
      nh_->getParam(obstacle_name + "/radius", radius);
      addSphere(name, radius, pose);
      break;

    case "box":
      double x; double y; double z;
      nh_->getParam(obstacle_name + "/x", x);
      nh_->getParam(obstacle_name + "/y", y);
      nh_->getParam(obstacle_name + "/z", z);
      addBox(name, x, y, z, pose);
      break;

    case "capsule":
      double radius; double halfLength
      nh_->getParam(obstacle_name + "/radius", radius);
      nh_->getParam(obstacle_name + "/halfLength", halfLength);
      addCapsule(name, radius, halfLength, pose);
      break;
    }
  obs_idx += 1;
 }
  // Adding the collision pairs to the geometry model

  std::vector<std::vector<std::string>> collision_pairs;
  nh_->getParam("collision_pairs", collision_pairs);
  
  for (std::vector<std::string> collision_pair : collision_pairs) {
    addCollisionPair(collision_pair[0], collision_pair[1])
  }

}

void ObstacleParamsParser::addSphere(const std::string &name,
                                     const double &radius,
                                     const Eigen::VectorXd &pose);
{
  Eigen::Vector3d translation(pose[0], pose[1], pose[2]);
  auto geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
      new hpp::fcl::Sphere(radius));
  pinocchio::SE3 obstacle_pose(Eigen::Quaterniond(1., 0., 0., 0.), translation);
  pinocchio::GeometryObject obstacle(name, 0, 0, geometry, obstacle_pose);
  collision_model_->addGeometryObject(obstacle);
}

void ObstacleParamsParser::addBox(const std::string &name, const double &x,
                                  const double &y, const double &z,
                                  const Eigen::VectorXd &pose);
{
  Eigen::Vector3d translation(pose[0], pose[1], pose[2]);
  Eigen::Quaterniond rotation(pose[3], pose[4], pose[5], pose[6]);

  auto geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
      new hpp::fcl::Box(x, y, z));
  pinocchio::SE3 obstacle_pose(rotation, translation);
  pinocchio::GeometryObject obstacle(name, 0, 0, geometry, obstacle_pose);
  collision_model_->addGeometryObject(obstacle);
}

void ObstacleParamsParser::addCapsule(const std::string &name,
                                      const double &radius,
                                      const double &halfLength,
                                      const Eigen::VectorXd &pose);
{
  Eigen::Vector3d translation(pose[0], pose[1], pose[2]);
  Eigen::Quaterniond rotation(pose[3], pose[4], pose[5], pose[6]);
  auto geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
      new hpp::fcl::Caspule(radius, halfLength));
  pinocchio::SE3 obstacle_pose(rotation, translation);
  pinocchio::GeometryObject obstacle(name, 0, 0, geometry, obstacle_pose);
  collision_model_->addGeometryObject(obstacle);
}

void ObstacleParamsParser::addCollisionPair(const std::string &name_object1,
                                            const std::string &name_object2);
{
  std::size_t object1Id = collision_model_->getGeometryId(name_object1);
  std::size_t object2Id = collision_model_->getGeometryId(name_object2);
  if (!collision_model_->existGeometryName(name_object1) ||
      !collision_model_->existGeometryName(name_object2)) {
    std::cerr << "Object ID not found for collision pair: " << object1Id
              << " and " << object2Id << std::endl;
  }

  collision_model_->addCollisionPair(
      pinocchio::CollisionPair(object1Id, object2Id));
}
};
} // namespace panda_torque_mpc
