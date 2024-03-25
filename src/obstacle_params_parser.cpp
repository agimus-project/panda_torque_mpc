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
    const boost::shared_ptr<ros::NodeHandle> &pnh,
    const boost::shared_ptr<pinocchio::GeometryModel> &collision_model)
    : pnh_(pnh), collision_model_(collision_model){};

ObstacleParamsParser::~ObstacleParamsParser(){};

void ObstacleParamsParser::addCollisions() {
  // First adding the obstacles to the geometry model
  
  std::cout << "in the parser" << std::endl;

  int obs_idx = 1;

  std::cout << "has param: " << "obstacle" + std::to_string(obs_idx) + "/type" << " 0 for no, 1 for yes: " << pnh_->hasParam("obstacle" + std::to_string(obs_idx) + "/type") << std::endl;
  while (pnh_->hasParam("obstacle" + std::to_string(obs_idx) + "/type")) {

    std::string obstacle_name;
    obstacle_name = "obstacle" + std::to_string(obs_idx);


    std::cout << "Obstacle_name: " << obstacle_name << std::endl;
    std::string type;
    if (!pnh_->hasParam(obstacle_name + "/type")) {
      std::cerr << "No obstacle type declared." << std::endl;
    }
    pnh_->getParam(obstacle_name + "/type", type);
    std::cout << "type : " << type << std::endl;
    std::vector<double> translation_vect;
    pnh_->getParam(obstacle_name + "/translation", translation_vect);
    const auto translation =
        Eigen::Map<Eigen::Vector3d>(translation_vect.data());

    std::vector<double> rotation_vect;
    pnh_->getParam(obstacle_name + "/rotation", rotation_vect);
    const auto rotation = Eigen::Map<Eigen::Quaterniond>(rotation_vect.data());

    pinocchio::GeometryObject::CollisionGeometryPtr geometry;

    if (type == "sphere") {
      double radius;
      pnh_->getParam(obstacle_name + "/radius", radius);
      geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
          new hpp::fcl::Sphere(radius));
    } else if (type == "box") {
      double x;
      double y;
      double z;
      pnh_->getParam(obstacle_name + "/x", x);
      pnh_->getParam(obstacle_name + "/y", y);
      pnh_->getParam(obstacle_name + "/z", z);
      geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
          new hpp::fcl::Box(x, y, z));
    } else if (type == "capsule") {
      double radius;
      double halfLength;
      pnh_->getParam(obstacle_name + "/radius", radius);
      pnh_->getParam(obstacle_name + "/halfLength", halfLength);
      geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
          new hpp::fcl::Capsule(radius, halfLength));
    } else {
      std::cerr << "No type or wrong type in the obstacle config. Try to use "
                   "the one implemented, such as 'sphere', 'box' or 'capsule'."
                << std::endl;
      return;
    }
    pinocchio::SE3 obstacle_pose(rotation, translation);
    pinocchio::GeometryObject obstacle(obstacle_name, 0, 0, geometry,
                                       obstacle_pose);
    collision_model_->addGeometryObject(obstacle);
    obs_idx += 1;
  }
  // Adding the collision pairs to the geometry model

XmlRpc::XmlRpcValue collision_pairs;

if (pnh_->getParam("collision_pairs", collision_pairs)) {
    if (collision_pairs.getType() == XmlRpc::XmlRpcValue::TypeArray && collision_pairs.size() > 0) {
        for (int i = 0; i < collision_pairs.size(); ++i) {
            if (collision_pairs[i].getType() == XmlRpc::XmlRpcValue::TypeArray && collision_pairs[i].size() == 2) {
                if (collision_pairs[i][0].getType() == XmlRpc::XmlRpcValue::TypeString &&
                    collision_pairs[i][1].getType() == XmlRpc::XmlRpcValue::TypeString) {
                    std::string object1 = static_cast<std::string>(collision_pairs[i][0]);
                    std::string object2 = static_cast<std::string>(collision_pairs[i][1]);
                    std::cout << "object1: " << object1 << std::endl; 
                    std::cout << "object2: " << object2 << std::endl; 

                    addCollisionPair(object1, object2);
                } else {
                    std::cerr << "Invalid collision pair type." << std::endl;
                    return;
                }
            } else {
                std::cerr << "Invalid collision pair number." << std::endl;
                return;
            }
        }
    } else {
        std::cerr << "No collision pair." << std::endl;
        return;
    }
}
}


void ObstacleParamsParser::addCollisionPair(const std::string &name_object1,
                                            const std::string &name_object2) {
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
}; // namespace panda_torque_mpc
   // namespace panda_torque_mpc
