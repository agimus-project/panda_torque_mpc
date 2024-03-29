#pragma once

#include "panda_torque_mpc/obstacle_params_parser.h"

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

ObstacleParamsParser::ObstacleParamsParser(
    const boost::shared_ptr<ros::NodeHandle> &pnh,
    const boost::shared_ptr<pinocchio::GeometryModel> &collision_model)
    : pnh_(pnh), collision_model_(collision_model){};

ObstacleParamsParser::~ObstacleParamsParser(){};

void ObstacleParamsParser::addCollisions() {
  // First adding the obstacles to the geometry model

  int obs_idx = 1;

  while (pnh_->hasParam("obstacle" + std::to_string(obs_idx) + "/type")) {

    std::string obstacle_name = "obstacle" + std::to_string(obs_idx);

    std::string type;
    pnh_->getParam(obstacle_name + "/type", type);
    std::vector<double> translation_vect;

    if (!pnh_->hasParam(obstacle_name + "/translation")) {
      std::cerr << "No obstacle translation declared for the obstacle named: " << obstacle_name <<  std::endl;
      return;
    }
    pnh_->getParam(obstacle_name + "/translation", translation_vect);
    const auto translation =
        Eigen::Map<Eigen::Vector3d>(translation_vect.data());

    std::vector<double> rotation_vect;
    if (!pnh_->hasParam(obstacle_name + "/rotation")) {
      std::cerr << "No obstacle rotation declared for the obstacle named: " << obstacle_name << std::endl;
      return;
    }
    pnh_->getParam(obstacle_name + "/rotation", rotation_vect);
    const auto rotation = Eigen::Map<Eigen::Quaterniond>(rotation_vect.data());

    pinocchio::GeometryObject::CollisionGeometryPtr geometry;

    if (type == "sphere") {
      double radius;
      if (pnh_->hasParam(obstacle_name + "/radius")) {
        pnh_->getParam(obstacle_name + "/radius", radius);
        geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
            new hpp::fcl::Sphere(radius));
      } else {
        std::cerr << "No dimension or wrong dimensions in the obstacle "
                     "config. Try to use "
                     "the ones for the shapes desired, such as 'radius' for "
                     "'sphere','x', 'y', 'z' for 'box' or 'halfLength' and "
                     "'radius' for 'capsule'."
                  << std::endl;
        return;
      }
    } else if (type == "box") {
      double x;
      double y;
      double z;
      if (pnh_->hasParam(obstacle_name + "/x") &&
          pnh_->hasParam(obstacle_name + "/y") &&
          pnh_->hasParam(obstacle_name + "/z") ) {
        pnh_->getParam(obstacle_name + "/x", x);
        pnh_->getParam(obstacle_name + "/y", y);
        pnh_->getParam(obstacle_name + "/z", z);
        geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
            new hpp::fcl::Box(x, y, z));
      } else {
        std::cerr << "No dimension or wrong dimensions in the obstacle "
                     "config. Try to use "
                     "the ones for the shapes desired, such as 'radius' for "
                     "'sphere','x', 'y', 'z' for 'box' or 'halfLength' and "
                     "'radius' for 'capsule'."
                  << std::endl;
        return;
      }

    } else if (type == "capsule") {
      double radius;
      double halfLength;
      if (pnh_->hasParam(obstacle_name + "/radius") &&
          pnh_->hasParam(obstacle_name + "/halfLength")) {
        pnh_->getParam(obstacle_name + "/radius", radius);
        pnh_->getParam(obstacle_name + "/halfLength", halfLength);
        geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
            new hpp::fcl::Capsule(radius, halfLength));
      }
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
    obs_idx ++;
  }
  // Adding the collision pairs to the geometry model

  XmlRpc::XmlRpcValue collision_pairs;

  if (pnh_->getParam("collision_pairs", collision_pairs)) {
    if (collision_pairs.getType() == XmlRpc::XmlRpcValue::TypeArray &&
        collision_pairs.size() > 0) {
      for (int i = 0; i < collision_pairs.size(); ++i) {
        if (collision_pairs[i].getType() == XmlRpc::XmlRpcValue::TypeArray &&
            collision_pairs[i].size() == 2) {
          if (collision_pairs[i][0].getType() ==
                  XmlRpc::XmlRpcValue::TypeString &&
              collision_pairs[i][1].getType() ==
                  XmlRpc::XmlRpcValue::TypeString) {
            std::string object1 =
                static_cast<std::string>(collision_pairs[i][0]);
            std::string object2 =
                static_cast<std::string>(collision_pairs[i][1]);

            addCollisionPair(object1, object2);
          } else {
            std::cerr << "Invalid collision pair type for collision pair " << i << std::endl;
            return;
          }
        } else {
          std::cerr << "Invalid collision pair number for collision pair " <<i << std::endl;
          return;
        }
      }
    } else {
      std::cout << "No collision pair." << std::endl;
    }
  }
}

void ObstacleParamsParser::addCollisionPair(const std::string &name_object1,
                                            const std::string &name_object2) {
  const std::size_t object1Id = collision_model_->getGeometryId(name_object1);
  const std::size_t object2Id = collision_model_->getGeometryId(name_object2);
  if (!collision_model_->existGeometryName(name_object1) ||
      !collision_model_->existGeometryName(name_object2)) {
    std::cerr << "Object ID not found for collision pair: " << object1Id
              << " and " << object2Id << std::endl;
  }

  collision_model_->addCollisionPair(
      pinocchio::CollisionPair(object1Id, object2Id));
}
}; // namespace panda_torque_mpc
