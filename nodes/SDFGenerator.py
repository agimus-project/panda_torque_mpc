import math
import random

class SDFGenerator:
    def __init__(self) -> None:
        self._available_colors = [
            "Gazebo/Red",
            "Gazebo/Blue",
            "Gazebo/Green",
            "Gazebo/Yellow",
            "Gazebo/Orange",
            "Gazebo/Purple",
            "Gazebo/White",
            "Gazebo/Black",
            "Gazebo/Grey",
            "Gazebo/DarkGrey",
            "Gazebo/Turquoise",
            "Gazebo/Indigo"
        ]

    def _generate_random_color(self) -> str:
        """
        Generates a random color from the available colors in Gazebo materials.

        Returns:
            str: Random color name.
        """
        return random.choice(self._available_colors)

    def generate_box(self, name: str, size: float) -> str:
        """      
        Generates SDF syntax for a box.

        Parameters:
            name (str): Name of the box.
            size (tuple): Size of the box in (width, height, depth).
            
        Returns:
            str: SDF syntax for the box.
        """
        # Generate random color
        color = self._generate_random_color()

        sdf = f"""
            <sdf version='1.4'>
            <model name="{name}">
            <static>true</static>
            <pose>0 0 0 0 0 0</pose>
            <link name="{name}_link">
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>{size[0]} {size[1]} {size[2]}</size>
                        </box>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>{color}</name>
                        </script>
                    </material>
                </visual>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>{size[0]} {size[1]} {size[2]}</size>
                        </box>
                    </geometry>
                </collision>
            </link>
        </model>
        """
        return sdf

    def generate_sphere(self, name: str, radius: float) -> str:
        """
        Generates SDF syntax for a sphere.

        Parameters:
            name (str): Name of the sphere.
            radius (float): Radius of the sphere.
            translation (tuple): Translation of the sphere in (x, y, z).
            rotation (tuple): Rotation of the sphere in (roll, pitch, yaw) (if quaternion is not provided).
            quaternion (tuple): Quaternion array in (x, y, z, w) format (if rotation is not provided).

        Returns:
            str: SDF syntax for the sphere.
        """           
        color = self._generate_random_color()

        sdf = f"""
        <sdf version='1.4'>
        <model name="{name}">
            <static>true</static>
            <link name="{name}_link">
                <pose>0 0 0 0 0 0</pose>
                <visual name="visual">
                    <geometry>
                        <sphere>
                            <radius>{radius}</radius>
                        </sphere>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>{color}</name>
                        </script>
                    </material>
                </visual>
                <collision name="collision">
                    <geometry>
                        <sphere>
                            <radius>{radius}</radius>
                        </sphere>
                    </geometry>
                </collision>
            </link>
        </model>
        """
        return sdf

    def generate_cylinder(self, name: str, radius: float, halfLength: float) -> str:
        """
        Generates SDF syntax for a cylinder.

        Parameters:
            name (str): Name of the cylinder.
            radius (float): Radius of the cylinder.
            helfLength (float): halfLength of the cylinder.

        Returns:
            str: SDF syntax for the cylinder.
        """
        # Generate random color
        color = self._generate_random_color()

        sdf = f"""
        <sdf version='1.4'>
        <model name="{name}">
            <static>true</static>
            <pose>0 0 0 0 0 0</pose>
            <link name="{name}_link">                    
                <visual name="visual">
                    <geometry>
                        <cylinder>
                            <radius>{radius}</radius>
                            <length>{halfLength *2}</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>{color}</name>
                        </script>
                    </material>
                </visual>
                <collision name="collision">
                    <geometry>
                        <cylinder>
                            <radius>{radius}</radius>
                            <length>{halfLength *2}</length>
                        </cylinder>
                    </geometry>
                </collision>
            </link>
        </model>
        """
        return sdf      


    def generate_capsule(self, name: str, radius: float, halfLength: float) -> str:
        """
        Generates SDF syntax for a capsule.

        Parameters:
            name (str): Name of the capsule.
            radius (float): Radius of the capsule.
            length (float): Length of the capsule.

        Returns:
            str: SDF syntax for the capsule.
        """
        color = self._generate_random_color()

        sdf = f"""
        <sdf version='1.4'>
        <model name="{name}">
            <static>true</static>
            <pose>0 0 0 0 0 0</pose>
            <link name="{name}_link">
            
                <!-- First sphere -->
                <visual name="visual_sphere1">
                    <pose>0 0 {halfLength} 0 0 0</pose>
                    <geometry>
                        <sphere>
                            <radius>{radius}</radius>
                        </sphere>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>{color}</name>
                        </script>
                    </material>
                </visual>
                <collision name="collision_sphere1">
                <pose>0 0 {halfLength} 0 0 0</pose>
                    <geometry>
                        <sphere>
                            <radius>{radius}</radius>
                        </sphere>
                    </geometry>
                </collision>

                <!-- Cylinder -->
                <visual name="visual_cylinder">
                    <geometry>
                        <cylinder>
                            <radius>{radius}</radius>
                            <length>{halfLength * 2}</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>{color}</name>
                        </script>
                    </material>
                </visual>
                <collision name="collision_cylinder">
                    <geometry>
                        <cylinder>
                            <radius>{radius}</radius>
                            <length>{halfLength * 2}</length>
                        </cylinder>
                    </geometry>
                </collision>

                <!-- Second sphere -->
                <visual name="visual_sphere2">
                    <pose>0 0 -{halfLength} 0 0 0</pose> <!-- Move the sphere to the end of the capsule -->
                    <geometry>
                        <sphere>
                            <radius>{radius}</radius>
                        </sphere>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>{color}</name>
                        </script>
                    </material>
                </visual>
                <collision name="collision_sphere2">
                    <pose>0 0 -{halfLength} 0 0 0</pose> <!-- Move the sphere to the end of the capsule -->
                    <geometry>
                        <sphere>
                            <radius>{radius}</radius>
                        </sphere>
                    </geometry>
                </collision>
            </link>
        </model>
        """
        return sdf
