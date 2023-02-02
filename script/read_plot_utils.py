import numpy as np

from pathlib import Path
from rosbags.dataframe import get_dataframe
from rosbags.highlevel import AnyReader

# def index_2_tarr(index):
#     # Compute time indices
#     idx_arr = df_q.index.to_numpy()
#     t_delta_arr = idx_arr - idx_arr[0] 
#     ns2sec = np.vectorize(lambda x: float(x)/1e9)
#     return ns2sec(t_delta_arr)

# def trim_dataframes()


def read_jsid_bag(bag_path, controller_name):

  topics = [
    '/{}/joint_configurations_comparison'.format(controller_name), 
    '/{}/joint_velocities_comparison'.format(controller_name), 
    '/{}/joint_torques_comparison'.format(controller_name)
  ]

  fields = ['commanded', 'measured', 'error']

  with AnyReader([Path(bag_path)]) as reader:
      df_q   = get_dataframe(reader, topics[0], fields)
      df_dq  = get_dataframe(reader, topics[1], fields)
      df_tau = get_dataframe(reader, topics[2], fields)


  # there might be a one msg difference between the different topics in a same bag -> trim that
  min_size = len(min([df_q.index, df_dq.index, df_tau.index], key=lambda idx: len(idx))) 
  df_q = df_q[:min_size]
  df_dq = df_dq[:min_size]
  df_tau = df_tau[:min_size]

  # Errors
  q_err_arr   = np.asarray([l for l in df_q['error']])
  dq_err_arr  = np.asarray([l for l in df_dq['error']])
  tau_err_arr = np.asarray([l for l in df_tau['error']])

  # Compute time indices
  idx_arr = df_q.index.to_numpy()
  t_delta_arr = idx_arr - idx_arr[0] 
  ns2sec = np.vectorize(lambda x: float(x)/1e9)
  t_arr = ns2sec(t_delta_arr)

  return t_arr, q_err_arr, dq_err_arr, tau_err_arr
