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

  ~ObstacleParamsParser() {}

  // Loading the Yaml file
  YAML::Node config = YAML::LoadFile("config.yaml");

  const YAML::Node &obstacles = config["obstacles"];
  // Going through all the obstacles and adding them to the geom model
  for (const auto &obstacle : obstacles) {
  }
}

  void ObstacleParamsParser::addObstacle(const std::string &name, const std::string &type,
                   const Eigen::VectorXd &dim, const Eigen::VectorXd &pose);
{
  switch (type) {
  case "sphere":
    double radius = dim[0];
    addSphere(name, radius, pose);

  case "box":
    double x = dim[0];
    double y = dim[1];
    double z = dim[2];
    addBox(name, x, y, z, pose);

  case "capsule":
    double radius = dim[0];
    double halfLength = dim[1];
    addCapsule(name, radius, halfLength, pose);
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
};
} // namespace panda_torque_mpc
