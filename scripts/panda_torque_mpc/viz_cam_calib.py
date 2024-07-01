import numpy as np
import pinocchio as pin
from example_robot_data import load
import meshcat.geometry as g


# Pose of panda2_ref_camera_link in panda2_hand
# pose_panda_hand_camera:
xc = 0.13682768843416482
yc = -0.009443400298528096
zc = -0.08387833599341328
r = 0.022100209841200636
p = -1.3156624527412932
y = 3.061881150974905



# y = 0.022100209841200636
# r = -1.3156624527412932
# p = 3.061881150974905

eRcl = pin.rpy.rpyToMatrix(r,p,y)
eMcl = pin.SE3.Identity()
eMcl.rotation = eRcl
eMcl.translation = np.array([xc, yc, zc])


r = load('panda')
viz = pin.visualize.MeshcatVisualizer(r.model, r.collision_model, r.visual_model)
viz.initViewer(loadModel=True)

ee_frame = 'panda_hand'
ee_id = r.model.getFrameId(ee_frame)
oMe = r.framePlacement(r.q0,ee_id,update_kinematics=True)

# camera link - camera color (from tf)
# - Translation: [-0.001, 0.015, 0.000]
# - Rotation: in Quaternion [-0.498, 0.504, -0.494, 0.505]
pose_c_color = [-0.001, 0.015, 0.000,  -0.498, 0.504, -0.494, 0.505]
cMcolor = pin.XYZQUATToSE3(pose_c_color)

eMc = eMcl * cMcolor
print('pose_e_c', pin.SE3ToXYZQUAT(eMc))

# display color camera frame in world frame
oMc = oMe * eMcl * cMcolor
frame_axes = g.triad(0.15)
viz.viewer['camera_frame'].set_object(frame_axes)
viz.viewer['camera_frame'].set_transform(oMc.homogeneous)

viz.display(r.q0)