"""Combines the various pieces of the MoveIt wrapper classes."""

from motion_planning_interface.MotionPlanner import MotionPlanner
from motion_planning_interface.PlanningScene import PlanningSceneM

from moveit_msgs.msg import PlanningScene
from motion_planning_interface.Gripper import Gripper


class MotionPlanningInterface:
    """Combines the various pieces of the MoveIt wrapper classes."""

    def __init__(self, node):
        """Initialize the classes needed for MoveIt.

        Args:
            node (Node): The parent node to use for interfacing with ROS
        """
        self.node = node
        self.mp = MotionPlanner(self.node, 'fer_arm')

        self.ps = PlanningSceneM(self.node)
        self.gripper = Gripper(self.node, 'hand')

    # wait for Robot State to work on this function
    def saveCurrentJoints(self):
        pass

    def saveJoints(self, namedConfig, jointData):
        """Save a dictionary of joints under a named configuration in a text file.

        Args:
            namedConfig (str): The name of the configuration to save.
            jointList (_type_): Dictionary of joints, excluding fer_finger_joints

        jointData = {
            'fer_joint1': 0.2669241215141775,
            'fer_joint2': 0.17016910129487495,
            'fer_joint3': 0.36917465657013154,
            'fer_joint4': -0.9936450053395579,
            'fer_joint5': -0.066977404914001,
            'fer_joint6': 1.1533546347023933,
            'fer_joint7': 1.389501659278778,
        }
        """
        fileName = (
            'src/motion_planning_interface/motion_planning_interface/saved_joints/'
            + namedConfig
            + '.txt'
        )
        with open(fileName, 'a') as f:
            for key, value in jointData.items():
                f.write(f"{key}, {value}\n")

    def loadJoints(self, namedConfig):
        """Load a dictionary of joints under a named configuration from a text file.

        Args:
            namedConfig (str): The name of the configuration to load.

        Returns:
            dict: A dictionary containing joint names as keys and joint values as floats.
        """
        fileName = (
            'src/motion_planning_interface/motion_planning_interface/saved_joints/'
            + namedConfig
            + '.txt'
        )
        jointData = {}

        try:
            with open(fileName, 'r') as f:
                for line in f:
                    key, value = line.strip().split(', ')
                    jointData[key] = float(value)
        except FileNotFoundError:
            print(f"Error: The file '{fileName}' does not exist.")
        except Exception as e:
            print(f"Error loading joints: {e}")

        return jointData
