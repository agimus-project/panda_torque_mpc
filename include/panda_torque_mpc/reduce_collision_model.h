#pragma once

#include <boost/smart_ptr/shared_ptr.hpp>
#include <vector>

#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/fwd.hpp>

#include <hpp/fcl/shape/geometric_shapes.h>

/// Only works for the current PANDA URDF as it depends on the positions of the spheres/cylinders in the URDF.

namespace panda_torque_mpc {
  class ReduceCollisionModel {
  public:
    pinocchio::GeometryModel reduce_capsules_robot(
        const boost::shared_ptr<pinocchio::GeometryModel> &collision_model) {
      auto reduced_collision_model =*collision_model;
      std::vector<std::string> list_names_capsules;

      for (size_t geom_id = 0; geom_id < collision_model->ngeoms; geom_id++) {

        auto &geometry_object = collision_model->geometryObjects[geom_id];

        if (dynamic_cast<hpp::fcl::Cylinder*>(geometry_object.geometry.get()) != nullptr) {

          const size_t last_underscore_pos = geometry_object.name.rfind('_');
          const std::string geom_name = geometry_object.name;
          const std::string link_name = geom_name.substr(0, last_underscore_pos);

          if ((collision_model->existGeometryName(link_name + "_1") &&
              collision_model->existGeometryName(link_name + "_2")) ||
              (collision_model->existGeometryName(link_name + "_4") &&
              collision_model->existGeometryName(link_name + "_5"))) {
            
            std::string capsule_name, sphere1_name, sphere2_name;
            if (std::find(list_names_capsules.begin(), list_names_capsules.end(),
                          link_name + "_0") == list_names_capsules.end()) {

              /// If the cylinder has the name finishing by _0; then the two corresponding spheres have
              /// the names finishing by _1 and _2. That's how the URDF is made here.
              capsule_name = link_name + "_0";
              sphere1_name = link_name + "_1";
              sphere2_name = link_name + "_2";
            
            } else {
              /// Same logic here but if there is another capsule representing the link of the robot, 
              /// the second cylinder will have _3 at the end and _4 and _5 for the spheres.
              capsule_name = link_name + "_1";
              sphere1_name = link_name + "_4";
              sphere2_name = link_name + "_5";

            }
            list_names_capsules.push_back(capsule_name);

            const auto cylinder_geom = static_cast<hpp::fcl::Cylinder*>(geometry_object.geometry.get());
            const auto geometry = pinocchio::GeometryObject::CollisionGeometryPtr(
                new hpp::fcl::Capsule(cylinder_geom->radius,
                                      cylinder_geom->halfLength));

            const pinocchio::GeometryObject capsule(
                capsule_name, geometry_object.parentJoint,
                geometry_object.parentFrame, geometry, geometry_object.placement);

            reduced_collision_model.addGeometryObject(capsule);
            reduced_collision_model.removeGeometryObject(geom_name);
            reduced_collision_model.removeGeometryObject(sphere1_name);
            reduced_collision_model.removeGeometryObject(sphere2_name);
          }
        }
      }
      return reduced_collision_model;
    }
  };
} // namespace panda_torque_mpc
