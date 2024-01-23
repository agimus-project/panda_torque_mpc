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
        pinocchio_model_dir = join(dirname(str(abspath(__file__))), "models")
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
        OBSTACLE_POSE = pin.SE3.Identity()
        OBSTACLE_POSE.translation = np.array([0.0, 0.0, 0.825])
        OBSTACLE = hppfcl.Sphere(0.35/2.0)
        OBSTACLE_GEOM_OBJECT = pin.GeometryObject(
            "obstacle",
            rmodel.getFrameId("universe"),
            rmodel.frames[rmodel.getFrameId("universe")].parent,
            OBSTACLE,
            OBSTACLE_POSE,
        )
        ID_OBSTACLE_PA = cmodel.addGeometryObject(OBSTACLE_GEOM_OBJECT)
        
        cmodel.addCollisionPair(
            pin.CollisionPair(
                cmodel.getGeometryId("panda2_rightfinger_0"),
                cmodel.getGeometryId("obstacle"),
            )
        )
        cmodel.addCollisionPair(
            pin.CollisionPair(
                cmodel.getGeometryId("panda2_leftfinger_0"),
                cmodel.getGeometryId("obstacle"),
            )
        )
        cmodel.addCollisionPair(
            pin.CollisionPair(
                cmodel.getGeometryId("panda2_link7_sc_1"),
                cmodel.getGeometryId("obstacle"),
            )
        )
        cmodel.addCollisionPair(
            pin.CollisionPair(
                cmodel.getGeometryId("panda2_link7_sc_4"),
                cmodel.getGeometryId("obstacle"),
            )
        )


        return rmodel, cmodel
    
    
if __name__=="__main__":
    rmodel, cmodel = load_panda()