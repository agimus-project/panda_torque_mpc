import numpy as np
import pinocchio as pin
from pathlib import Path
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
plt.rcParams['pdf.fonttype'] = 42  # avoid pbe with fonts when submitting paperplaza


TOPIC_POSE_BC_REF = '/ctrl_mpc_linearized/cam_pose_ref_viz'  # msg_pose_bc_ref
TOPIC_POSE_BC = '/ctrl_mpc_linearized/cam_pose_ref_viz_bis'  # msg_pose_bc

LINEWIDTH = 3.0

bags_folder = Path('../bags/mpc_simu_expes/')


# EXPE_BATCH 1
id_exp_lst = [5,10,20,30,50,80]
# id_exp_lst = [5,10,20,30,40,50,60,70,80]
EXPES = {
    'names': [f'mpc_expe_wf_{id_exp:02}' for id_exp in id_exp_lst],
    'labels': [fr'$w_v={id_exp}$' for id_exp in id_exp_lst]
}



assert len(EXPES['names']) == len(EXPES['labels'])

fig_t, ax_t = plt.subplots(figsize=(6.4*1.4,4.8))
fig_o, ax_o = plt.subplots(figsize=(6.4*1.4,4.8))
fig_t.canvas.manager.set_window_title('error tracking translation') 
fig_o.canvas.manager.set_window_title('error tracking orientation') 

N_EXPES = len(EXPES['names'])
for k in range(N_EXPES):
    bag_path = bags_folder / f"{EXPES['names'][k]}.bag"

    b = bagreader(bag_path.as_posix())

    # replace the topic name as per your need
    pose_bc_ref_msg = b.message_by_topic(TOPIC_POSE_BC_REF)
    pose_bc_msg = b.message_by_topic(TOPIC_POSE_BC)
    df_bc_ref = pd.read_csv(pose_bc_ref_msg)
    df_bc = pd.read_csv(pose_bc_msg)

    max_l = min(len(df_bc_ref), len(df_bc))

    df_bc_ref = df_bc_ref.iloc[:max_l]
    df_bc = df_bc.iloc[:max_l]

    t0 = min(df_bc_ref.Time.iloc[0], df_bc.Time.iloc[0])

    ts_pose_ref = df_bc_ref.Time - t0
    ts_pose = df_bc_ref.Time - t0

    cols_pose = ['pose.position.x', 'pose.position.y', 'pose.position.z', 
                'pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z', 'pose.orientation.w']

    pose_bc_ref_arr = df_bc_ref[cols_pose].to_numpy()
    pose_bc_arr = df_bc[cols_pose].to_numpy()


    T_bc_ref_lst = [pin.XYZQUATToSE3(pose) for pose in pose_bc_ref_arr]
    T_bc_lst = [pin.XYZQUATToSE3(pose) for pose in pose_bc_arr]


    err_t_xyz = [T_bc_ref.translation - T_bc.translation for T_bc_ref, T_bc in zip(T_bc_ref_lst, T_bc_lst)]
    err_o_xyz = [pin.log3(T_bc_ref.rotation.T@T_bc.rotation) for T_bc_ref, T_bc in zip(T_bc_ref_lst, T_bc_lst)]

    err_t = np.linalg.norm(err_t_xyz, axis=1)
    err_o = np.linalg.norm(err_o_xyz, axis=1)
    err_o = np.rad2deg(err_o)

    # Detect moment where we start to receive data for each rosbag and remove what's before
    first_non_zero_idx = np.where(err_t > 0.0001)[0][0]

    # Remove end of the rosbag to synchhronize experiments
    Ncut = 300
    ts_pose_ref = ts_pose_ref[first_non_zero_idx:Ncut]
    ts_pose_ref = ts_pose_ref - ts_pose_ref.iloc[0]
    err_t = err_t[first_non_zero_idx:Ncut]
    err_o = err_o[first_non_zero_idx:Ncut]

    ax_t.plot(ts_pose_ref, 1000*err_t, lw=LINEWIDTH, label=EXPES['labels'][k])
    ax_o.plot(ts_pose_ref, err_o, lw=LINEWIDTH, label=EXPES['labels'][k])
 

font = {'fontname':'serif'}

ax_t.set_xlabel('Time [s]', fontsize=18, **font)
ax_t.set_ylabel('Translation Error [mm]', fontsize=18, **font)
ax_o.set_xlabel('Time [s]', fontsize=18, **font)
ax_o.set_ylabel('Orientation Error [deg]', fontsize=18, **font)
    
ax_t.tick_params(axis='x', labelsize=14)
ax_t.tick_params(axis='y', labelsize=14)
ax_o.tick_params(axis='x', labelsize=14)
ax_o.tick_params(axis='y', labelsize=14)

ax_t.grid()
ax_o.grid()


for fig in [fig_t, fig_o]:
    fig.legend(loc='upper center', bbox_to_anchor=(0.6, 0.99),
            ncol=2, fancybox=True, shadow=True, fontsize=14)
    fig.subplots_adjust(bottom=0.13)

fig_t.savefig('err_t_wv.pdf')
fig_o.savefig('err_o_wv.pdf')

plt.show()