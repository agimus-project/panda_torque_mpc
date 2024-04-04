import os
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
import example_robot_data
import pinocchio as pin

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


targets = {}
targets["x"] = [0] * 240
targets["y1"] = [-0.35] * 240
targets["y2"] = [0.35] * 240
targets["z"] = [1] * 240
for i_field, field in enumerate(fields):
    fig_dq, ax_dq = plt.subplots(1, 1)
    fig_tau, ax_tau = plt.subplots(1, 1)
    fig_q, ax_q = plt.subplots(3, 1)
    fig_errq, ax_errq = plt.subplots(3, 1)

    # fig_q.canvas.manager.set_window_title(f"Joint configurations {field}")
    fig_dq.canvas.manager.set_window_title(f"Joint velocities {field}")
    fig_tau.canvas.manager.set_window_title(f"Joint torques {field}")

    d_res = read_jsid_bag(BAG_PATHS[0], CONTROLLER_NAME)
    if len(d_res["t"]) < 600:
        start_idx = 0
        end_idx = 240
    else:
        start_idx = 240
        end_idx = 480
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
    # d_res = read_jsid_bag(BAG_PATHS[1], CONTROLLER_NAME)
    # for i in range(7):
    #     if not JOINTS_TO_PLOT[i]:
    #         continue
    #     c = COLORS[i]
    #     sym = 'x'
    #     ax_q.plot(d_res['t'], d_res['q'][field][:, i],
    #               f'{c}{sym}', label=f'q{i}', markersize=MSIZE)
    #     ax_dq.plot(d_res['t'], d_res['dq'][field][:, i],
    #                f'{c}{sym}', label=f'dq{i}', markersize=MSIZE)
    #     ax_tau.plot(d_res['t'], d_res['tau'][field][:, i],
    #                 f'{c}{sym}', label=f'tau{i}', markersize=MSIZE)

    # ax_q.set_title(field)

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
