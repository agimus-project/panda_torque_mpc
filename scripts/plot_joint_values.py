import os
import numpy as np
import pandas as pd
import example_robot_data
import pinocchio as pin

import matplotlib.pyplot as plt

from read_plot_utils import read_jsid_bag

DIRECTORY = "."

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

fields = ["measured"]


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
robot.model = model


def get_pose_list(q_array):
    pose_list = np.zeros((q_array.shape[0], 3))

    for i, q in enumerate(q_array):
        pose_list[i, :] = robot.placement(q.copy(), robot.nq).translation.copy()
        pose_list[i, 2] -= 0.107  # distance between last joint and end-effector

    return pose_list


cycle_len = 800
targets = {}
targets["x"] = [0] * cycle_len
targets["y1"] = [-0.35] * cycle_len
targets["y2"] = [0.35] * cycle_len
targets["z"] = [1] * cycle_len

data = read_jsid_bag(BAG_PATHS[0], CONTROLLER_NAME)


def plotting():
    for i_field, field in enumerate(fields):
        fig_q, ax_q = plt.subplots(robot.nq, 1)
        fig_dq, ax_dq = plt.subplots(robot.nq, 1)
        fig_tau, ax_tau = plt.subplots(robot.nq, 1)
        fig_xyz, ax_xyz = plt.subplots(3, 1)
        fig_err_xyz, ax_err_xyz = plt.subplots(3, 1)
        # fig_it_dur, ax_it_dur = plt.subplots(1, 1)
        fig_q.canvas.manager.set_window_title(f"Joint positions {field}")
        fig_dq.canvas.manager.set_window_title(f"Joint velocities {field}")
        fig_tau.canvas.manager.set_window_title(f"Joint torques {field}")
        fig_xyz.canvas.manager.set_window_title(f"End effector position {field}")
        fig_err_xyz.canvas.manager.set_window_title(
            f"End effector position error {field}"
        )
        # fig_it_dur.canvas.manager.set_window_title("FDDP iteration duration")

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
        """ax_it_dur.plot(
            data["t"][:cycle_len],
            data["iter_duration"][:cycle_len],
            markersize=MSIZE,
        )"""

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
            ax_q[i].plot(
                data["t"][:cycle_len],
                data["q"][field][:cycle_len, i],
                # f"{c}{sym}",
                # label=f"dq{i}",
                markersize=MSIZE,
            )
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
        ax_q[0].set_title(field)
        ax_dq[0].set_title(field)
        ax_xyz[0].set_title(field)
        ax_err_xyz[0].set_title(field)
        ax_tau[0].set_title(field)
        for i in range(robot.nq):
            ax_q[i].set_xlabel("t (s)")
            ax_dq[i].set_xlabel("t (s)")
            ax_tau[i].set_xlabel("t (s)")
            ax_q[i].set_ylabel(f"q{i} (rad)")
            ax_dq[i].set_ylabel(f"dq{i} (rad/s)")
            ax_tau[i].set_ylabel(f"tau{i} (N.m)")
            ax_q[i].grid()
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
        # ax_it_dur.set_xlabel("t (s)")
    plt.show()


plotting()


"""

# fields = ['error', 'measured']
# fields = ['error']

for i_field, field in enumerate(fields):
    fig_dq, ax_dq = plt.subplots(1, 1)
    fig_tau, ax_tau = plt.subplots(1, 1)
    fig_q, ax_q = plt.subplots(1, 1)

    fig_q.canvas.manager.set_window_title(f"Joint configurations {field}")
    fig_dq.canvas.manager.set_window_title(f"Joint velocities {field}")
    fig_tau.canvas.manager.set_window_title(f"Joint torques {field}")

    d_res = read_jsid_bag(BAG_PATHS[0], CONTROLLER_NAME)
    for i in range(7):
        if not JOINTS_TO_PLOT[i]:
            continue
        c = COLORS[i]
        sym = "."
        ax_q.plot(
            d_res["t"],
            d_res["q"][field][:, i],
            f"{c}{sym}",
            label=f"q{i}",
            markersize=MSIZE,
        )
        ax_dq.plot(
            d_res["t"],
            d_res["dq"][field][:, i],
            f"{c}{sym}",
            label=f"dq{i}",
            markersize=MSIZE,
        )
        ax_tau.plot(
            d_res["t"],
            d_res["tau"][field][:, i],
            f"{c}{sym}",
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

    ax_q.set_title(field)
    ax_dq.set_title(field)
    ax_tau.set_title(field)
    ax_q.set_xlabel("t (s)")
    ax_dq.set_xlabel("t (s)")
    ax_tau.set_xlabel("t (s)")
    ax_q.set_ylabel("q (rad)")
    ax_dq.set_ylabel("dq (rad/s)")
    ax_tau.set_ylabel("tau (N.m)")

    ax_q.grid()
    ax_dq.grid()
    ax_tau.grid()
    ax_q.legend()
    ax_dq.legend()
    ax_tau.legend()

plt.show()
"""
