import os
import numpy as np

import mpc_utils
import pin_utils
import example_robot_data
import pinocchio as pin
from read_plot_utils import read_jsid_bag
import yaml

DIRECTORY = "."  # ../bags/

CONTROLLER_NAME = "ctrl_mpc_linearized"
BAG_NAMES = [
    f"{CONTROLLER_NAME}_expe.bag",
]


BAG_PATHS = [os.path.join(DIRECTORY, name) for name in BAG_NAMES]


def get_pose_ref(pose_ref, time):
    first_publish_time = pose_ref["time"][0]
    cycle_duration_2 = pose_ref["time"][1] - pose_ref["time"][0]
    cycle_duration = cycle_duration_2 * 2
    first_target = pin.XYZQUATToSE3(pose_ref["pose"][1])
    second_target = pin.XYZQUATToSE3(pose_ref["pose"][0])
    pose_ref = []
    for t in time:
        if (
            t % cycle_duration < first_publish_time
            or t % cycle_duration > first_publish_time + cycle_duration_2
        ):
            pose_ref.append(first_target)
        else:
            pose_ref.append(second_target)
    return pose_ref


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
robot_model = pin.buildReducedModel(model, locked_joints, q0)
robot.model = robot_model


data = read_jsid_bag(BAG_PATHS[0], CONTROLLER_NAME)
pose_ref = get_pose_ref(data["pose_ref"], data["t"])
"""
q0 = data["xu_solution"]["q"][:, 0, 0]
cycle_len = 800
plt.plot(data["t"][:cycle_len], q0[:cycle_len])
plt.show()"""
# # # # # # # # # # # #
###  MPC SIMULATION ###
# # # # # # # # # # # #
# OCP parameters
ocp_params = {}
ocp_params["N_h"] = params["nb_shooting_nodes"] - 1
ocp_params["dt"] = params["dt_ocp"]
ocp_params["maxiter"] = 4
ocp_params["pin_model"] = robot_model
ocp_params["armature"] = params["armature"]
ocp_params["id_endeff"] = robot_model.getFrameId("panda_joint7")
ocp_params["active_costs"] = []

# Simu parameters
sim_params = {}
dt_sim = (data["t"][-1] - data["t"][0]) / len(data["t"])
sim_params["sim_freq"] = int(1 / dt_sim)
sim_params["mpc_freq"] = int(1 / dt_sim)

sim_params["T_sim"] = 8
log_rate = 100
# Initialize simulation data
x0 = np.concatenate(
    [data["xu_solution"]["q"][0, 0, :], data["xu_solution"]["v"][0, 0, :]]
)
robot.x0 = x0
sim_data = mpc_utils.init_sim_data(sim_params, ocp_params, robot.x0)

mpc_cycle = 0
for i in range(sim_data["N_sim"]):

    if i % log_rate == 0:
        print("\n SIMU step " + str(i) + "/" + str(sim_data["N_sim"]) + "\n")

    # Solve OCP if we are in a planning cycle (MPC/planning frequency)
    if i % int(sim_params["sim_freq"] / sim_params["mpc_freq"]) == 0:
        # Set x0 to measured state

        x0 = np.concatenate(
            [data["xu_solution"]["q"][i, :, :], data["xu_solution"]["v"][i, :, :]]
        )

        # Warm start using previous solution
        xs = np.concatenate(
            [data["xu_solution"]["q"][i, :, :], data["xu_solution"]["v"][i, :, :]],
            axis=1,
        )
        us = data["xu_solution"]["u"][i, :, :]
        sim_data["state_pred"][mpc_cycle, :, :] = xs
        sim_data["ctrl_pred"][mpc_cycle, :, :] = us
        # Extract relevant predictions for interpolations
        x_curr = sim_data["state_pred"][
            mpc_cycle, 0, :
        ]  # x0* = measured state    (q^,  v^ )
        x_pred = sim_data["state_pred"][
            mpc_cycle, 1, :
        ]  # x1* = predicted state   (q1*, v1*)
        u_curr = sim_data["ctrl_pred"][
            mpc_cycle, 0, :
        ]  # u0* = optimal control   (tau0*)
        # Record costs references
        q = sim_data["state_pred"][mpc_cycle, 0, : sim_data["nq"]]
        sim_data["ctrl_ref"][mpc_cycle, :] = data["xu_solution"]["u"][i, 0, :]
        sim_data["state_ref"][mpc_cycle, :] = np.concatenate(
            [data["xu_solution"]["q"][i, 0, :], data["xu_solution"]["v"][i, 0, :]]
        )
        sim_data["lin_pos_ee_ref"][mpc_cycle, :] = pose_ref[i].translation
        # m0=croco_reaching.solver.problem.runningModels[0].differential.costs
        # Select reference control and state for the current MPC cycle
        x_ref_MPC_RATE = x_curr + sim_data["ocp_to_mpc_ratio"] * (x_pred - x_curr)
        u_ref_MPC_RATE = u_curr
        if mpc_cycle == 0:
            sim_data["state_des_MPC_RATE"][mpc_cycle, :] = x_curr
        sim_data["ctrl_des_MPC_RATE"][mpc_cycle, :] = u_ref_MPC_RATE
        sim_data["state_des_MPC_RATE"][mpc_cycle + 1, :] = x_ref_MPC_RATE

        # Increment planning counter
        mpc_cycle += 1

        # Select reference control and state for the current SIMU cycle
        x_ref_SIM_RATE = x_curr + sim_data["ocp_to_mpc_ratio"] * (x_pred - x_curr)
        u_ref_SIM_RATE = u_curr

        # First prediction = measurement = initialization of MPC
        if i == 0:
            sim_data["state_des_SIM_RATE"][i, :] = x_curr
        sim_data["ctrl_des_SIM_RATE"][i, :] = u_ref_SIM_RATE
        sim_data["state_des_SIM_RATE"][i + 1, :] = x_ref_SIM_RATE

        sim_data["state_mea_SIM_RATE"][i + 1, :] = np.concatenate(
            [
                data["xu_solution"]["q"][i + 1, 0, :],
                data["xu_solution"]["v"][i + 1, 0, :],
            ]
        )  # x_mea_SIM_RATE


plot_data = mpc_utils.extract_plot_data_from_sim_data(sim_data)

mpc_utils.plot_mpc_results(
    plot_data,
    which_plots=["all"],
    PLOT_PREDICTIONS=True,
    pred_plot_sampling=int(sim_params["mpc_freq"] / 10),
)
