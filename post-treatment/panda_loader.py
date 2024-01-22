from os.path import dirname, join, abspath
import numpy as np

import pinocchio as pin
import hppfcl

def load_panda():
        """Load the robot from the models folder.

        Returns:
            rmodel, vmodel, cmodel: Robot model, visual model & collision model of the robot.
        """

        ### LOADING THE ROBOT
        pinocchio_model_dir = join
            (dirname(str(abspath(__file__))), "models"
        )
        model_path = join(pinocchio_model_dir, "franka_description/robots")
        mesh_dir = pinocchio_model_dir
        urdf_filename = "franka2.urdf"
        urdf_model_path = join(join(model_path, "panda"), urdf_filename)

        rmodel, cmodel, vmodel = pin.buildModelsFromUrdf(
            urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
        )

        q0 = pin.neutral(rmodel)

        rmodel, [vmodel, cmodel] = pin.buildReducedModel(
            rmodel, [vmodel, cmodel], [1, 9, 10], q0
        )

        ### CREATING THE SPHERE ON THE UNIVERSE
        CAPSULE_POSE = pin.SE3.Identity()
        CAPSULE_POSE.translation = np.array([0.0, 0.0, 0.825])
        CAPSULE = hppfcl.Sphere(0.35/2.0)
        CAPSULE_GEOM_OBJECT = pin.GeometryObject(
            "CAPSULE",
            rmodel.getFrameId("universe"),
            rmodel.frames[rmodel.getFrameId("universe")].parentJoint,
            CAPSULE,
            CAPSULE_POSE,
        )
        ID_CAPSULE_PA = cmodel.addGeometryObject(CAPSULE_GEOM_OBJECT)

        return rmodel, cmodel