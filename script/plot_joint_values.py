import os
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

from read_plot_utils import read_jsid_bag

CONTROLLER_NAME = 'joint_space_ID_controller'
# DIRECTORY = '/home/mfourmy/Downloads/franka_experiments_31_01_23'

# BAG_NAME = 'joint_space_ID_controller_franka_lowstiff.bag'
# YML_NAME = 'joint_space_ID_controller_franka_lowstiff.yaml'
# BAG_NAME = 'joint_space_ID_controller_franka_midstiff.bag'
# YML_NAME = 'joint_space_ID_controller_franka_midstiff.yaml'
# BAG_NAME = 'joint_space_ID_controller_franka_stiff.bag'
# YML_NAME = 'joint_space_ID_controller_franka_stiff.yaml'
# BAG_NAME = 'joint_space_ID_controller_pin_lowstiff.bag'
# YML_NAME = 'joint_space_ID_controller_pin_lowstiff.yaml'
# BAG_NAME = 'joint_space_ID_controller_pin_midstiff.bag'
# YML_NAME = 'joint_space_ID_controller_pin_midstiff.yaml'
# BAG_NAME = 'joint_space_ID_controller_pin_stiff.bag'
# YML_NAME = 'joint_space_ID_controller_pin_stiff.yaml'


DIRECTORY = '../bags/'
BAG_NAMES = [
  'joint_space_ID_controller_LONG_expe.bag',
  # 'joint_space_ID_controller_lowK_NoSat.bag',
  'joint_space_ID_controller_lowK_Sat.bag',
]

BAG_PATHS = [os.path.join(DIRECTORY, name) for name in BAG_NAMES]



fig_dq, ax_dq = plt.subplots(1,1)
fig_tau, ax_tau = plt.subplots(1,1)
fig_q, ax_q = plt.subplots(1,1)


fig_q.canvas.manager.set_window_title('Joint configurations')
fig_dq.canvas.manager.set_window_title('Joint velocities')
fig_tau.canvas.manager.set_window_title('Joint torques')



JOINTS_TO_PLOT = [1,1,1,1,1,1,1]
COLORS = 'rgbcmyk'
MSIZE = 3
t_arr, q_err_arr, dq_err_arr, tau_err_arr = read_jsid_bag(BAG_PATHS[0], CONTROLLER_NAME)
for i in range(7):
    if not JOINTS_TO_PLOT[i]: continue
    c = COLORS[i]
    sym = '.'
    ax_q.plot(t_arr, q_err_arr[:,i], f'{c}{sym}', label=f'q{i}', markersize=MSIZE)
    ax_dq.plot(t_arr, dq_err_arr[:,i], f'{c}{sym}', label=f'dq{i}', markersize=MSIZE)
    ax_tau.plot(t_arr, tau_err_arr[:,i], f'{c}{sym}', label=f'tau{i}', markersize=MSIZE)

t_arr, q_err_arr, dq_err_arr, tau_err_arr = read_jsid_bag(BAG_PATHS[1], CONTROLLER_NAME)
for i in range(7):
    if not JOINTS_TO_PLOT[i]: continue
    c = COLORS[i]
    sym = 'x'
    ax_q.plot(t_arr, q_err_arr[:,i], f'{c}{sym}', label=f'q{i}', markersize=MSIZE)
    ax_dq.plot(t_arr, dq_err_arr[:,i], f'{c}{sym}', label=f'dq{i}', markersize=MSIZE)
    ax_tau.plot(t_arr, tau_err_arr[:,i], f'{c}{sym}', label=f'tau{i}', markersize=MSIZE)


ax_q.set_title('error')
ax_dq.set_title('error')
ax_tau.set_title('error')
ax_q.set_xlabel('t (s)')
ax_dq.set_xlabel('t (s)')
ax_tau.set_xlabel('t (s)')
ax_q.set_ylabel('q (rad)')
ax_dq.set_ylabel('dq (rad/s)')
ax_tau.set_ylabel('tau (N.m)')

ax_q.grid()
ax_dq.grid()
ax_tau.grid()
ax_q.legend()
ax_dq.legend()
ax_tau.legend()

plt.show()