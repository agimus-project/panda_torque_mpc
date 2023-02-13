import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib

font = {'family' : 'normal',
        'weight' : 'normal',
        'size'   : 22}

matplotlib.rc('font', **font)

path = '/home/ros/.ros/test_panfa_time_vec_120s.csv'

df = pd.read_csv(path, sep=",")
t_arr = df['t'].to_numpy()
t0 =  t_arr[0]
t_arr = t_arr - t0
t_shift_arr = np.roll(t_arr,1)
dt_arr = t_arr - t_shift_arr
dur_arr = df['dur'].to_numpy()


plt.figure('t')
plt.plot(t_arr[1:], dt_arr[1:], '.', label='t(k) - t(k-1)', markersize=2)
plt.plot(t_arr[1:], dur_arr[1:], '.', label='dur', markersize=2)
plt.legend()
plt.grid()
plt.xlabel('t (s)')
plt.ylabel('dt (s)')
plt.title('Delta t between 2 update calls')

plt.show()