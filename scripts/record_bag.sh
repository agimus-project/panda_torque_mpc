rosbag record \
    --chunksize=1048576 \
    /ctrl_mpc_linearized/ocp_solve_time \
    /ctrl_mpc_linearized/robot_sensors \
    /ctrl_mpc_linearized/motion_capture_pose_ref \
    /ctrl_mpc_linearized/motion_server_control \
    /ctrl_mpc_linearized/absolute_pose_ref