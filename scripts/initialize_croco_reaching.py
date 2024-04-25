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

    robot_model_reduced = pin.buildReducedModel(robot.model, locked_joints, robot.q0)
    robot.model = robot_model_reduced
    """
    urdf_path = "../urdf/robot.urdf"
    srdf_path = "../srdf/demo.srdf"
    model = pin.Model()
    pin.urdf.buildModel(urdf_path, model)
    pin.srdf.loadReferenceConfigurations(model, srdf_path, False)"""
    # q0 = model.referenceConfigurations["default"]
    # model = pin.buildReducedModel(model, locked_joints, q0)
    croco_reaching = CrocoddylReaching(robot.model, robot.collision_model, params)

    xs_0 = [
        0.00108776,
        -0.00102196,
        0.0086259,
        -0.0717591,
        0.00711182,
        0.308149,
        0.133417,
        3.51826e-07,
        3.13603e-05,
        -6.84773e-05,
        -5.4034e-05,
        0.00102681,
        0.000522924,
        0.00203523,
    ]

    xs_init = (params["nb_shooting_nodes"] + 1) * [xs_0]
    tau_grav = pin.computeGeneralizedGravity(
        robot.model, robot.data, np.array(xs_0[:7])
    )
    tau_grav = tau_grav[:7]
    us_init = params["nb_shooting_nodes"] * [tau_grav]

    return croco_reaching, xs_init, us_init, params
