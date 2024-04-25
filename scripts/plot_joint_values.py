import os
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
import mpc_utils
import pin_utils
import example_robot_data
import pinocchio as pin
import initialize_croco_reaching as croco_reach
from panda_torque_mpc_pywrap import CrocoddylReaching
from read_plot_utils import read_jsid_bag

DIRECTORY = "."  # ../bags/

# CONTROLLER_NAME = 'ctrl_joint_space_ID'
# BAG_NAMES = [
#   f'{CONTROLLER_NAME}_expe.bag',
# ]

CONTROLLER_NAME = "ctrl_mpc_linearized"
BAG_NAMES = [
    f"{CONTROLLER_NAME}_expe.bag",
]


BAG_PATHS = [os.path.join(DIRECTORY, name) for name in BAG_NAMES]


JOINTS_TO_PLOT = [1, 1, 1, 1, 1, 1, 1]
COLORS = "rgbcmyk"
MSIZE = 5


fields = ["measured"]  # "error", "measured","commanded"
# fields = ['error', 'measured']
# fields = ['error']
robot = example_robot_data.load("panda")
locked_joints = [
    robot.model.getJointId("panda_finger_joint1"),
    robot.model.getJointId("panda_finger_joint2"),
]
robot_model_reduced = pin.buildReducedModel(robot.model, locked_joints, robot.q0)
robot.model = robot_model_reduced


def get_pose_list(q_array):
    pose_list = np.zeros((q_array.shape[0], 3))

    for i, q in enumerate(q_array):
        pose_list[i, :] = robot.placement(q.copy(), robot.nq).translation.copy()
        pose_list[i, 0] = -1 * pose_list[i, 0] + 0.563
        pose_list[i, 1] = -1 * pose_list[i, 1] - 0.165
        pose_list[i, 2] += 0.674
    return pose_list


cycle_len = 300
targets = {}
targets["x"] = [0] * cycle_len
targets["y1"] = [-0.35] * cycle_len
targets["y2"] = [0.35] * cycle_len
targets["z"] = [1] * cycle_len

d_res = read_jsid_bag(BAG_PATHS[0], CONTROLLER_NAME)


def plotting():
    for i_field, field in enumerate(fields):
        fig_dq, ax_dq = plt.subplots(1, 1)
        fig_tau, ax_tau = plt.subplots(1, 1)
        fig_q, ax_q = plt.subplots(3, 1)
        fig_errq, ax_errq = plt.subplots(3, 1)

        # fig_q.canvas.manager.set_window_title(f"Joint configurations {field}")
        fig_dq.canvas.manager.set_window_title(f"Joint velocities {field}")
        fig_tau.canvas.manager.set_window_title(f"Joint torques {field}")

        if len(d_res["t"]) < 600:
            start_idx = 0
            end_idx = cycle_len
        else:
            start_idx = cycle_len
            end_idx = 2 * cycle_len
        pose_list = get_pose_list(d_res["q"][field])
        axes = ["x", "y", "z"]
        for i in range(3):
            if not JOINTS_TO_PLOT[i]:
                continue
            c = COLORS[i]
            sym = "."

            ax_q[i].plot(
                d_res["t"][start_idx:end_idx],
                pose_list[start_idx:end_idx, i],
                # f"{c}{sym}",
                label=axes[i],
                markersize=MSIZE,
            )
        for i, key in enumerate(targets.keys()):

            if i > 1:
                i = i - 1
            ax_q[i].plot(
                d_res["t"][start_idx:end_idx],
                targets[key],
                label="target_" + key,
                markersize=MSIZE,
            )
            ax_errq[i].plot(
                d_res["t"][start_idx:end_idx],
                targets[key] - pose_list[start_idx:end_idx, i],
                label="target_" + key,
                markersize=MSIZE,
            )

        for i in range(7):
            if not JOINTS_TO_PLOT[i]:
                continue
            c = COLORS[i]
            sym = "."
            ax_dq.plot(
                d_res["t"][start_idx:end_idx],
                d_res["dq"][field][start_idx:end_idx, i],
                # f"{c}{sym}",
                label=f"dq{i}",
                markersize=MSIZE,
            )
            ax_tau.plot(
                d_res["t"][start_idx:end_idx],
                d_res["tau"][field][start_idx:end_idx, i],
                # f"{c}{sym}",
                label=f"tau{i}",
                markersize=MSIZE,
            )

        ax_dq.set_title(field)
        ax_tau.set_title(field)
        ax_dq.set_xlabel("t (s)")
        ax_tau.set_xlabel("t (s)")
        ax_dq.set_ylabel("dq (rad/s)")
        ax_tau.set_ylabel("tau (N.m)")

        ax_dq.grid()
        ax_tau.grid()
        ax_dq.legend()
        ax_tau.legend()

        for i, axe in enumerate(axes):
            ax_q[i].set_xlabel("t (s)")
            ax_q[i].set_ylabel(axe)
            ax_q[i].grid()
            ax_q[i].legend()
            ax_errq[i].set_xlabel("t (s)")
            ax_errq[i].set_ylabel("error" + axe)
            ax_errq[i].grid()
            ax_errq[i].legend()

    plt.show()


croco_reaching, xs_init, us_init, params = croco_reach.get_croco_reaching()
first_solve = True
# # # # # # # # # # # #
###  MPC SIMULATION ###
# # # # # # # # # # # #
# OCP parameters
ocp_params = {}
ocp_params["N_h"] = params["nb_shooting_nodes"]
ocp_params["dt"] = params["dt_ocp"]
ocp_params["maxiter"] = 1
ocp_params["pin_model"] = robot.model
ocp_params["armature"] = params["armature"]
ocp_params["id_endeff"] = robot.model.getFrameId("panda_joint7")
ocp_params["active_costs"] = [
    "translation_cost",
    "placement_cost",
    "velocity_cost",
    "state_reg",
    "ctrl_reg",
]
# ocp_params["active_costs"] = solver.problem.runningModels[
#   0
# ].differential.costs.active.tolist()

# Simu parameters
sim_params = {}
sim_params["sim_freq"] = 30
sim_params["mpc_freq"] = 30
sim_params["T_sim"] = 15
log_rate = 100
# Initialize simulation data
robot.x0 = xs_init[0]
sim_data = mpc_utils.init_sim_data(sim_params, ocp_params, robot.x0)
# Display target
# mpc_utils.display_ball(endeff_translation, RADIUS=0.05, COLOR=[1.0, 0.0, 0.0, 0.6])
# Simulate
mpc_cycle = 0
for i in range(sim_data["N_sim"]):

    if i % log_rate == 0:
        print("\n SIMU step " + str(i) + "/" + str(sim_data["N_sim"]) + "\n")

    # Solve OCP if we are in a planning cycle (MPC/planning frequency)
    if i % int(sim_params["sim_freq"] / sim_params["mpc_freq"]) == 0:
        # Set x0 to measured state

        x0 = np.concatenate([d_res["q"]["measured"][i], d_res["dq"]["measured"][i]])
        # Warm start using previous solution
        if first_solve:
            first_solve = False
        else:
            xs_init = croco_reaching.solver.xs
            # xs_init[0] = sim_data["state_mea_SIM_RATE"][i, :]
            us_init = croco_reaching.solver.us

        # Solve OCP & record MPC predictions
        # breakpoint()
        for idx in range(len(us_init)):
            xs_init[idx] = np.array(xs_init[idx])
            us_init[idx] = np.array(us_init[idx])
        xs_init[-1] = np.array(xs_init[-1])
        # croco_reaching.solving(x0, xs_init, us_init, d_res["t"][i])
        croco_reaching.solver.problem.x0 = x0
        time = d_res["t"][i]
        croco_reaching.set_ee_ref_placement(time, True, 1.0)

        croco_reaching.set_posture_ref(x0)

        croco_reaching.solve(xs_init, us_init)
        xs = croco_reaching.solver.xs
        us = croco_reaching.solver.us
        sim_data["state_pred"][mpc_cycle, :, :] = np.array(xs)
        sim_data["ctrl_pred"][mpc_cycle, :, :] = np.array(us)
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
        sim_data["ctrl_ref"][mpc_cycle, :] = pin_utils.get_u_grav(
            q,
            croco_reaching.solver.problem.runningModels[0].differential.pinocchio,
            ocp_params["armature"],
        )
        sim_data["state_ref"][mpc_cycle, :] = (
            croco_reaching.solver.problem.runningModels[0]
            .differential.costs.costs["state_reg"]
            .cost.residual.reference
        )
        sim_data["lin_pos_ee_ref"][mpc_cycle, :] = (
            croco_reaching.solver.problem.runningModels[0]
            .differential.costs.costs["translation_cost"]
            .cost.residual.reference
        )

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

        # Send torque to simulator & step simulator
        """
        robot_simulator.send_joint_command(u_ref_SIM_RATE)
        env.step()
        # Measure new state from simulator
        q_mea_SIM_RATE, v_mea_SIM_RATE = robot_simulator.get_state()
        # Update pinocchio model
        robot_simulator.forward_robot(q_mea_SIM_RATE, v_mea_SIM_RATE)
        # Record data
        x_mea_SIM_RATE = np.concatenate([q_mea_SIM_RATE, v_mea_SIM_RATE]).T"""
        sim_data["state_mea_SIM_RATE"][i + 1, :] = x0  # x_mea_SIM_RATE


plot_data = mpc_utils.extract_plot_data_from_sim_data(sim_data)

mpc_utils.plot_mpc_results(
    plot_data,
    which_plots=["all"],
    PLOT_PREDICTIONS=True,
    pred_plot_sampling=int(sim_params["mpc_freq"] / 10),
)
