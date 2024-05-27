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
croco_reaching, params, robot_model = croco_reach.get_croco_reaching()

robot = example_robot_data.load("panda")
robot.model = robot_model


def get_pose_list(q_array):
    pose_list = np.zeros((q_array.shape[0], 3))

    for i, q in enumerate(q_array):
        pose_list[i, :] = robot.placement(q.copy(), robot.nq).translation.copy()
        pose_list[i, 2] -= 0.107  # distance between last joint and end-effector

    return pose_list


cycle_len = 1000
targets = {}
targets["x"] = [0] * cycle_len
targets["y1"] = [-0.35] * cycle_len
targets["y2"] = [0.35] * cycle_len
targets["z"] = [1] * cycle_len

data = read_jsid_bag(BAG_PATHS[0], CONTROLLER_NAME)
data["t"] += data["target_time"][0]  # 34.18


def plotting():
    for i_field, field in enumerate(fields):
        fig_dq, ax_dq = plt.subplots(robot.nq, 1)
        fig_tau, ax_tau = plt.subplots(robot.nq, 1)
        fig_xyz, ax_xyz = plt.subplots(3, 1)
        fig_err_xyz, ax_err_xyz = plt.subplots(3, 1)
        fig_it_dur, ax_it_dur = plt.subplots(1, 1)

        fig_dq.canvas.manager.set_window_title(f"Joint velocities {field}")
        fig_tau.canvas.manager.set_window_title(f"Joint torques {field}")
        fig_xyz.canvas.manager.set_window_title(f"End effector position {field}")
        fig_err_xyz.canvas.manager.set_window_title(
            f"End effector position error {field}"
        )
        fig_it_dur.canvas.manager.set_window_title("FDDP iteration duration")

        cycle_len
        pose_list = get_pose_list(data["q"][field])
        axes = ["x", "y", "z"]
        for i in range(3):
            if not JOINTS_TO_PLOT[i]:
                continue
            c = COLORS[i]
            sym = "."

            ax_xyz[i].plot(
                data["t"][:cycle_len],
                pose_list[:cycle_len, i],
                # f"{c}{sym}",
                label=axes[i],
                markersize=MSIZE,
            )
        ax_it_dur.plot(
            data["t"][:cycle_len],
            data["iter_duration"][:cycle_len],
            markersize=MSIZE,
        )

        # plot targets for the end effector position xyz, plot errors wrt targets
        for i, key in enumerate(targets.keys()):
            if i > 1:
                i = i - 1
            ax_xyz[i].plot(
                data["t"][:cycle_len],
                targets[key],
                label="target_" + key,
                markersize=MSIZE,
            )
            ax_err_xyz[i].plot(
                data["t"][:cycle_len],
                targets[key] - pose_list[:cycle_len, i],
                label="target " + key,
                markersize=MSIZE,
            )

        for i in range(robot.nq):
            if not JOINTS_TO_PLOT[i]:
                continue
            c = COLORS[i]
            sym = "."
            ax_dq[i].plot(
                data["t"][:cycle_len],
                data["dq"][field][:cycle_len, i],
                # f"{c}{sym}",
                # label=f"dq{i}",
                markersize=MSIZE,
            )
            ax_tau[i].plot(
                data["t"][:cycle_len],
                data["tau"][field][:cycle_len, i],
                # f"{c}{sym}",
                # label=f"tau{i}",
                markersize=MSIZE,
            )
        ax_it_dur.set_title("FDDP iteration duration")
        ax_dq[0].set_title(field)
        ax_xyz[0].set_title(field)
        ax_err_xyz[0].set_title(field)
        ax_tau[0].set_title(field)
        for i in range(robot.nq):
            ax_dq[i].set_xlabel("t (s)")
            ax_tau[i].set_xlabel("t (s)")
            ax_dq[i].set_ylabel(f"dq{i} (rad/s)")
            ax_tau[i].set_ylabel(f"tau{i} (N.m)")
            ax_dq[i].grid()
            ax_tau[i].grid()

        for i, axe in enumerate(axes):
            ax_xyz[i].set_xlabel("t (s)")
            ax_xyz[i].set_ylabel(f"{axe} (m)")
            ax_xyz[i].grid()
            ax_xyz[i].legend()
            ax_err_xyz[i].set_xlabel("t (s)")
            ax_err_xyz[i].set_ylabel("error" + axe)
            ax_err_xyz[i].grid()
            ax_err_xyz[i].legend()
        ax_it_dur.set_xlabel("t (s)")
    plt.show()


plotting()
