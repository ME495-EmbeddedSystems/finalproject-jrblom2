"""Class for Interacting with the Planning Scene in rviz."""

#new method with matt and joe and liz 
from moveit_msgs.msg import (
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
)

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from rclpy.node import Node


class PlanningSceneM:
    """A class for manipulating the planning scene."""

    def __init__(self, node: Node):
        """Initialize the PlanningScene class to manage collision objects.

        Args:
            node (Node): The ROS2 node for communicating with the planning scene.
        """
        self.node = node
        self.scene_publisher = node.create_publisher(
            PlanningScene, 'monitored_planning_scene', 10
        )
        self.norm_publisher = node.create_publisher(
            PlanningScene, 'planning_scene', 10
        )
        self.collision_objects = {}  # To keep track of added objects
        self.planning_scene = PlanningScene()


    def publish_scene(self):
        """Publish the current planning scene to update in MoveIt."""
        self.planning_scene.is_diff = True
        self.norm_publisher.publish(self.planning_scene)
        self.scene_publisher.publish(self.planning_scene)
        self.node.get_logger().info(
            f"Published planning scene with {len(self.planning_scene.world.collision_objects)} objects"
        )


    def add_box(self, name: str, size: tuple, pose: Pose):
        """Add a box to the planning scene. This works.

        Args:
            name (str): Unique name for the box.
            size (tuple): Size of the box (x, y, z).
            pose (Pose): Pose of the box in the scene.
        """
        self.node.get_logger().info(f"Preparing to add box '{name}'")
        box = CollisionObject()
        box.id = name
        box.header.frame_id = 'base'
        box.header.stamp = self.node.get_clock().now().to_msg()
        box.operation = CollisionObject.ADD

        # Define the box shape
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size)  # [x, y, z]

        box.primitives = [primitive]
        box.primitive_poses = [pose]

        self.collision_objects[name] = box
        self.planning_scene.world.collision_objects.append(box)
        self.planning_scene.is_diff = True
        self.publish_scene()
        self.node.get_logger().info(f"published scene with box")


    def remove_box(self, name: str):
        """Remove a box from the planning scene by name. This is a work in progress.

        Args:
            name (str): The name of the box to remove.
        """
        for item in self.planning_scene.world.collision_objects:
            if item.id == name:
                self.node.get_logger().info(f"Removing box: {name}")
                
                # Create a new CollisionObject to remove
                box_to_remove = CollisionObject()
                box_to_remove.id = name
                box_to_remove.operation = CollisionObject.REMOVE

                # Add the removal operation to the planning scene
                self.planning_scene.world.collision_objects.append(box_to_remove)

                # Remove from the internal dictionary of collision objects
                if name in self.collision_objects:
                    del self.collision_objects[name]

                # Mark the scene as having a difference and publish
                self.planning_scene.is_diff = True
                self.publish_scene()
                self.node.get_logger().info(f"Published scene after removing box: {name}")
                break  # Exit after removing the first matching box
            else:
                self.node.get_logger().info(f"Box '{name}' not found in the scene.")

    def attach_object(self, name: str, link_name: str):
        """Attach an object to the robot's end-effector. This works

        Args:
            name (str): The name of the object to attach.
            link_name (str): The name of the link to attach the object to.
        """
        self.node.get_logger().info(
            f"Attempting to attach object '{name}' to link '{link_name}'"
        )

        # Find and remove the object from collision_objects
        found_object = None
        for item in self.planning_scene.world.collision_objects:
            if item.id == name:
                found_object = item
                self.planning_scene.world.collision_objects.remove(item)
                break

        if not found_object:
            self.node.get_logger().error(f"Object '{name}' not found in collision objects.")
            return

        # Create and attach the object to the robot
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object = found_object
        self.planning_scene.robot_state.attached_collision_objects.append(attached_object)

        # Publish the updated scene
        self.planning_scene.is_diff = True
        self.publish_scene()
        self.node.get_logger().info(f"Attached object '{name}' to link '{link_name}'.")

        self.node.get_logger().info(
            f"Current attached objects: {self.planning_scene.robot_state.attached_collision_objects}"
        )


    def detach_object(self, name: str):
            """Detach an object from the robot's end-effector.

            Args:
                name (str): The name of the object to detach.
            """
            self.node.get_logger().info(
                f"Attempting to detach object '{name}' from the robot."
            )

            # Find and remove the attached object from robot_state.attached_collision_objects
            attached_object = None
            for item in self.planning_scene.robot_state.attached_collision_objects:
                if item.object.id == name:
                    attached_object = item
                    self.planning_scene.robot_state.attached_collision_objects.remove(item)
                    self.node.get_logger().info(f"Detached object: {name} from robot.")
                    break

            if attached_object is not None:
                # After detaching, add the object back to the world collision objects
                self.node.get_logger().info(f"Adding object '{name}' back to the world.")
                self.planning_scene.world.collision_objects.append(attached_object.object)
                
            else:
                self.node.get_logger().info(f"Object '{name}' not found in attached objects.")

            # Publish the updated scene
            self.publish_scene()

            # Log the current state of attached objects
            self.node.get_logger().info(
                f"Current attached objects: {self.planning_scene.robot_state.attached_collision_objects}"
            )


    def load_scene_from_parameters(self, params):
        """Load objects into the planning scene from a parameter list.

        Args:
            params (list): List of dictionaries specifying object details.
        """
        for obj in params:
            name = obj['name']
            size = tuple(obj['size'])  # [x, y, z]
            pose = Pose()
            pose.position.x = obj['position'][0]
            pose.position.y = obj['position'][1]
            pose.position.z = obj['position'][2]
            pose.orientation.x = obj['orientation'][0]
            pose.orientation.y = obj['orientation'][1]
            pose.orientation.z = obj['orientation'][2]
            pose.orientation.w = obj['orientation'][3]
            self.add_box(name, size, pose)
