import os
import numpy as np
import pandas as pd

from pathlib import Path
from rosbags.dataframe import get_dataframe
from rosbags.highlevel import AnyReader
import matplotlib.pyplot as plt


directory = '../bags/'
bag_name = 'toto.bag'

bag_path = os.path.join(directory, bag_name)
topics = [
  '/joint_space_ID_controller/joint_configurations_comparison', 
  '/joint_space_ID_controller/joint_velocities_comparison', 
  '/joint_space_ID_controller/joint_torques_comparison'
]

fields = ['commanded', 'measured', 'error']

with AnyReader([Path(bag_path)]) as reader:
    df_q   = get_dataframe(reader, topics[0], fields)
    df_dq  = get_dataframe(reader, topics[1], fields)
    df_tau = get_dataframe(reader, topics[2], fields)

# Errors
q_err_arr   = np.asarray([l for l in df_q['error']])
dq_err_arr  = np.asarray([l for l in df_dq['error']])
tau_err_arr = np.asarray([l for l in df_tau['error']])

# Compute time indices
idx_arr = df_q.index.to_numpy()
t_delta_arr = idx_arr - idx_arr[0] 
ns2sec = np.vectorize(lambda x: float(x)/1e9)
t_arr = ns2sec(t_delta_arr)

fig_q, ax_q = plt.subplots(1,1)
fig_dq, ax_dq = plt.subplots(1,1)
fig_tau, ax_tau = plt.subplots(1,1)

to_plot = [1,1,0,1,1,0,1]
for i in range(7):
    if not to_plot[i]: continue
    ax_q.plot(t_arr, q_err_arr[:,i], label='q{}'.format(i))
    ax_dq.plot(t_arr, dq_err_arr[:,i], label='dq{}'.format(i))
    ax_tau.plot(t_arr, tau_err_arr[:,i], label='tau{}'.format(i))

fig_q.canvas.manager.set_window_title('Joint configurations')
fig_dq.canvas.manager.set_window_title('Joint velocities')
fig_tau.canvas.manager.set_window_title('Joint torques')


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