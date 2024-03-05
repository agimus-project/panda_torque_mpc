#pragma once

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <memory>
#include <vector>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

namespace panda_torque_mpc {

class ObstacleInfosPublisher {
  ObstacleInfosPublisher(
      const std::shared_ptr<ros::NodeHandle> &ph,
      const boost::shared_ptr<pinocchio::Model> &pin_model_,
      const boost::shared_ptr<pinocchio::GeometryModel> &collision_model_);
  ~ObstacleInfosPublisher();
  {
    // Loading the Yaml file
    YAML::Node config = YAML::LoadFile("config.yaml");

    const YAML::Node& obstacles = config["obstacles"];
    // Going through all the obstacles and adding them to the geom model
    for (const auto& obstacle : obstacles)
    {

    }
  }
  void addCollisions();
  {

  }

  void addObstacle(const std::string &name, const std::string &type,
                   const Eigen::VectorXd &dim, const Eigen::VectorXd &pose);
  {
    switch (type) {
        case "sphere":
        double radius = dim[0];
        Eigen::Vector3d translation(pose[0], pose[1], pose[2]);

        auto geometry = pinocchio::GeometryObject::CollisionGeometryPtr(new hpp::fcl::Sphere(radius));
        pinocchio::SE3 obstacle_pose(Eigen::Quaterniond (1.,0.,0.,0.), translation);
        pinocchio::GeometryObject obstacle(name, 0,0, geometry, obstacle_pose);
        collision_model_->addGeometryObject(obstacle);
    }  
  }


  void addCollisionPair(const std::string &name_object1,
                        const std::string &name_object2);
  {
    int objectId1 = collision_model_->getGeometryId(name_object1);
    int objectId1 = collision_model_->getGeometryId(name_object2);
    if (objectId1 > collision_model_->geometryObjects.size() || objectId2 == collision_model_->geometryObjects.size()) {
        std::cerr << "Object ID not found for collision pair: " << objectId1 << " and " << objectId2 << std::endl;
        return;
    }
    collision_model_->addCollisionPair(pinocchio::CollisionPair(objectId1,objectId2));

  };
}

} // namespace panda_torque_mpc
