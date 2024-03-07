#pragma once

#include "panda_torque_mpc/obstacle_params_parser.h"

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

  ObstacleParamsParser::ObstacleParamsParser(
      const std::shared_ptr<ros::NodeHandle> &nh,
      const boost::shared_ptr<pinocchio::Model> &pin_model,
      const boost::shared_ptr<pinocchio::GeometryModel> &collision_model)
      : nh_(nh), pin_model_(pin_model), collision_model_(collision_model){};

  ObstacleParamsParser::~ObstacleParamsParser() {};

void ObstacleParamsParser::addCollisions()
{
  // First adding the obstacles to the geometry model
 int obs_idx = 0;
 while (nh_->hasParam("obstacle_" + std::to_string(obs_idx))) {

  std::string obstacle_name;
  obstacle_name = "obstacle_" + std::to_string(obs_idx);
  
  std::string type;
  if (!nh_->hasParam(obstacle_name + "/type"))
{
    std::cerr << "No obstacle type declared." << std::endl;
}
  nh_->getParam(obstacle_name + "/type", type);
  

  std::vector<double> translation_vect;
  nh_->getParam(obstacle_name + "/translation", translation_vect);
  const auto translation = Eigen::Map<Eigen::Vector3d>(translation_vect.data()) ;

  std::vector<double> rotation_vect;
  nh_->getParam(obstacle_name + "/rotation", rotation_vect);
  const auto rotation = Eigen::Map<Eigen::Quaterniond>(rotation_vect.data()) ;
  
  pinocchio::GeometryObject::CollisionGeometryPtr geometry;

  if (type == "sphere") {
    double radius;
    nh_->getParam(obstacle_name + "/radius", radius);
    geometry = pinocchio::GeometryObject::CollisionGeometryPtr(new hpp::fcl::Sphere(radius));
}  else if (type == "box") {
    double x; double y; double z;
    nh_->getParam(obstacle_name + "/x", x);
    nh_->getParam(obstacle_name + "/y", y);
    nh_->getParam(obstacle_name + "/z", z);
    geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
      new hpp::fcl::Box(x, y, z));
}  else if (type == "box") {
    double radius; 
    double halfLength;
    nh_->getParam(obstacle_name + "/radius", radius);
    nh_->getParam(obstacle_name + "/halfLength", halfLength);
    geometry = pinocchio::GeometryObject::CollisionGeometryPtr(new hpp::fcl::Caspule(radius, halfLength));
} else 
{
std::cerr << "No type or wrong type in the obstacle config. Try to use the one implemented, such as 'sphere', 'box' or 'capsule'." << std::endl;
}
  pinocchio::SE3 obstacle_pose(rotation, translation);
  pinocchio::GeometryObject obstacle(obstacle_name, 0, 0, geometry, obstacle_pose);
  collision_model_->addGeometryObject(obstacle);
  obs_idx += 1;
 }
  // Adding the collision pairs to the geometry model

  std::vector<std::vector<std::string>> collision_pairs;
  nh_->getParam("collision_pairs", collision_pairs);
  
  for (const auto& collision_pair : collision_pairs) {
      if (collision_pair.size() == 2) {
          addCollisionPair(collision_pair[0], collision_pair[1]);
      } else {
          std::cerr << "Invalid collision pair number." << std::endl;
      }
  }

}


void ObstacleParamsParser::addCollisionPair(const std::string &name_object1,
                                            const std::string &name_object2)
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
 // namespace panda_torque_mpc
