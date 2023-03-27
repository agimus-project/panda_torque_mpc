import os
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

from read_plot_utils import read_tsid_bag

# CONTROLLER_NAME = 'ctrl_task_space_ID'
CONTROLLER_NAME = 'ctrl_mpc_croco'

DIRECTORY = '../bags/'
EXPE_NAMES = [
    # 'expe_ref',
    'expe',
    #   'stiffer',
]


BAG_NAMES = [f'{CONTROLLER_NAME}_{expe}.bag' for expe in EXPE_NAMES]
BAG_PATHS = [os.path.join(DIRECTORY, name) for name in BAG_NAMES]

print('Reading ', BAG_PATHS[0])
d_res = read_tsid_bag(BAG_PATHS[0], CONTROLLER_NAME)

# JOINTS_TO_PLOT = [1,1,1,1,1,1,1]
COLORS = 'rgbcmyk'
MSIZE = 5

fig_dx, ax_dx = plt.subplots(2, 1)
fig_tau, ax_tau = plt.subplots(1, 1)
fig_x, ax_x = plt.subplots(2, 1)

fig_x.canvas.manager.set_window_title('Task Pose')
fig_dx.canvas.manager.set_window_title('Task vel')
fig_tau.canvas.manager.set_window_title('Joint torques')


# FIELD = 'commanded'
# FIELD = 'measured'
FIELD = 'error'

# position/linear part
for i in range(3):
    c = COLORS[i]
    sym = '.'
    ax_x[0].plot(d_res['t'], d_res['x'][FIELD][:, i],
                 f'{c}{sym}', label=f'x{i}', markersize=MSIZE)
    ax_dx[0].plot(d_res['t'], d_res['dx'][FIELD][:, i],
                  f'{c}{sym}', label=f'dx{i}', markersize=MSIZE)


for i in range(3):
    c = COLORS[i]
    sym = '.'
    ax_x[1].plot(d_res['t'], d_res['x'][FIELD][:, 3+i],
                 f'{c}{sym}', label=f'x{i}', markersize=MSIZE)
    ax_dx[1].plot(d_res['t'], d_res['dx'][FIELD][:, 3+i],
                  f'{c}{sym}', label=f'dx{i}', markersize=MSIZE)


for i in range(7):
    c = COLORS[i]
    sym = '.'
    ax_tau.plot(d_res['t'], d_res['tau'][FIELD][:, i],
                f'{c}{sym}', label=f'tau{i}', markersize=MSIZE)

ax_x[0].set_title(FIELD)
ax_dx[0].set_title(FIELD)
ax_x[1].set_xlabel('t (s)')
ax_dx[1].set_xlabel('t (s)')
ax_x[0].set_ylabel('x position (m)')
ax_x[1].set_ylabel('x orientation (rad)')
ax_dx[0].set_ylabel('dx linear (m)')
ax_dx[1].set_ylabel('dq  angular(rad/s)')

ax_tau.set_title(FIELD)
ax_tau.set_xlabel('t (s)')
ax_tau.set_ylabel('tau (N.m)')

ax_x[0].grid()
ax_dx[0].grid()
ax_x[1].grid()
ax_dx[1].grid()
ax_tau.grid()
ax_x[0].legend()
ax_dx[0].legend()
ax_x[1].legend()
ax_dx[1].legend()
ax_tau.legend()

plt.show()
