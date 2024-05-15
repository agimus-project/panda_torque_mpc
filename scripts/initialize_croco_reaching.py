from panda_torque_mpc_pywrap import (
    CrocoddylReaching,
)  # , TargetsConfig, CrocoddylConfig
import example_robot_data
import pinocchio as pin
import yaml
import numpy as np


def get_croco_reaching():
    with open("../config/controller_configs.yaml", "r") as file:
        params = yaml.safe_load(file)
        params = params["ctrl_mpc_linearized"]

    robot = example_robot_data.load("panda")
    locked_joints = [
        robot.model.getJointId("panda_finger_joint1"),
        robot.model.getJointId("panda_finger_joint2"),
    ]

    urdf_path = "../urdf/robot.urdf"
    srdf_path = "../srdf/demo.srdf"
    model = pin.Model()
    pin.buildModelFromUrdf(urdf_path, model)
    pin.loadReferenceConfigurations(model, srdf_path, False)
    q0 = model.referenceConfigurations["default"]
    model = pin.buildReducedModel(model, locked_joints, q0)
    collision_model = pin.buildGeomFromUrdf(model, urdf_path, pin.COLLISION)
    croco_reaching = CrocoddylReaching(model, collision_model, params)

    return croco_reaching, params, model


# croco_reaching, params, model = get_croco_reaching()
