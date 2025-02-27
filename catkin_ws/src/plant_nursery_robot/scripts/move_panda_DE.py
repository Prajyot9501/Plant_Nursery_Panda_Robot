#!/usr/bin/env python3

# Student: Prajyot Patil
# Task: Multiple Pose Management with STL Collision Object
# Date: 27th Apr 2024
# Acknowledgements: ChatGPT, MoveIt Tutorials, ROS Documentation

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler
import os

from moveit_msgs.msg import CollisionObject, Mesh
from shape_msgs.msg import Mesh as ShapeMesh  # Alias to avoid confusion

class PoseManager(object):
    def __init__(self):
        super(PoseManager, self).__init__()

        # Initialize MoveIt Commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pose_manager_node', anonymous=True)

        # Initialize the Robot Commander
        self.robot = moveit_commander.RobotCommander()

        # Initialize the Planning Scene Interface
        self.scene = moveit_commander.PlanningSceneInterface()

        # Initialize the Move Group for the Panda arm
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Set planning parameters
        self.move_group.set_planning_time(15)  # Increased planning time for complex moves
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)

        # Set goal tolerances
        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_goal_orientation_tolerance(0.001)

        # Allow replanning to increase the odds of a solution
        self.move_group.allow_replanning(True)

        # Set the reference frame for pose targets
        self.reference_frame = "panda_link0"
        self.move_group.set_pose_reference_frame(self.reference_frame)

        # Get the name of the end-effector link
        self.end_effector_link = self.move_group.get_end_effector_link()
        rospy.loginfo(f"End effector link: {self.end_effector_link}")

        rospy.sleep(2)  # Allow some time for initialization

        # Define and add collision objects
        self.define_collision_objects()

    def define_poses(self):
        """
        Defines multiple predefined poses for the Panda robot.
        Returns a dictionary with pose names as keys and Pose objects as values.
        """
        poses = {}

        # Home Pose: Safe default position
        home_quat = quaternion_from_euler(0, 0, 0)
        home_pose = geometry_msgs.msg.Pose()
        home_pose.position.x = 0.0
        home_pose.position.y = 0.0
        home_pose.position.z = 0.4
        home_pose.orientation.x = home_quat[0]
        home_pose.orientation.y = home_quat[1]
        home_pose.orientation.z = home_quat[2]
        home_pose.orientation.w = home_quat[3]
        poses['home'] = home_pose

        # Pose A: Intermediate Position
        pose_a_quat = quaternion_from_euler(pi/2, 0, 0)
        pose_a = geometry_msgs.msg.Pose()
        pose_a.position.x = 0.3
        pose_a.position.y = -0.2
        pose_a.position.z = 0.5
        pose_a.orientation.x = pose_a_quat[0]
        pose_a.orientation.y = pose_a_quat[1]
        pose_a.orientation.z = pose_a_quat[2]
        pose_a.orientation.w = pose_a_quat[3]
        poses['pose_a'] = pose_a

        # Pose B: Target Position
        pose_b_quat = quaternion_from_euler(-pi/2, 0, pi/4)
        pose_b = geometry_msgs.msg.Pose()
        pose_b.position.x = 0.4
        pose_b.position.y = 0.3
        pose_b.position.z = 0.6
        pose_b.orientation.x = pose_b_quat[0]
        pose_b.orientation.y = pose_b_quat[1]
        pose_b.orientation.z = pose_b_quat[2]
        pose_b.orientation.w = pose_b_quat[3]
        poses['pose_b'] = pose_b

        # Add more poses as needed
        # Example:
        # pose_c_quat = quaternion_from_euler(0, pi/4, pi/2)
        # pose_c = geometry_msgs.msg.Pose()
        # pose_c.position.x = 0.5
        # pose_c.position.y = 0.0
        # pose_c.position.z = 0.5
        # pose_c.orientation.x = pose_c_quat[0]
        # pose_c.orientation.y = pose_c_quat[1]
        # pose_c.orientation.z = pose_c_quat[2]
        # pose_c.orientation.w = pose_c_quat[3]
        # poses['pose_c'] = pose_c

        return poses

    def add_collision_object(self, object_id, mesh_filename, pose, scale=(1.0, 1.0, 1.0)):
        """
        Adds a collision object to the planning scene using an STL file.

        :param object_id: Unique identifier for the object.
        :param mesh_filename: Name of the STL file.
        :param pose: geometry_msgs.msg.Pose specifying the object's pose.
        :param scale: Tuple specifying the scale of the object in x, y, z.
        """
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = self.reference_frame

        # Define the mesh resource
        mesh = Mesh()
        mesh.mesh_resource = f"package://plant_nursery_robot/models/{mesh_filename}"
        mesh.mesh_use_embedded_materials = True  # Use embedded materials if available

        collision_object.meshes = [mesh]
        collision_object.mesh_poses = [pose]
        collision_object.operation = CollisionObject.ADD

        # Set scale if necessary
        collision_object.scale.x, collision_object.scale.y, collision_object.scale.z = scale

        # Add the collision object to the planning scene
        self.scene.add_object(collision_object)

        rospy.loginfo(f"Added collision object '{object_id}' to the planning scene.")

        # Wait for the object to be added
        self.wait_for_state_update(object_id, object_is_known=True, object_is_attached=False, timeout=4)

    def define_collision_objects(self):
        """
        Defines and adds collision objects to the planning scene.
        """
        # Example: Adding an STL object named 'custom_object.stl'
        object_id = "custom_stl_object"
        mesh_filename = "custom_object.stl"  # Replace with your actual STL file name

        # Define the pose for the object
        object_pose = geometry_msgs.msg.Pose()
        object_pose.position.x = 0.5  # Adjust as needed
        object_pose.position.y = 0.0
        object_pose.position.z = 0.2
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0

        # Define scale if necessary
        scale = (1.0, 1.0, 1.0)  # Adjust scaling as needed

        # Add the collision object
        self.add_collision_object(object_id, mesh_filename, object_pose, scale=scale)

    def wait_for_state_update(self, object_id, object_is_known=False, object_is_attached=False, timeout=4):
        """
        Waits for the planning scene to update with the specified object state.

        :param object_id: Unique identifier for the object.
        :param object_is_known: Boolean indicating if the object should be known in the scene.
        :param object_is_attached: Boolean indicating if the object should be attached to the robot.
        :param timeout: Maximum time to wait for the state update.
        :return: Boolean indicating if the desired state was achieved.
        """
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the object is in attached objects
            attached_objects = self.scene.get_attached_objects([object_id])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the object is in the scene
            is_known = object_id in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (object_is_attached == is_attached) and (object_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def select_pose(self, poses):
        """
        Presents the user with a list of available poses and returns the selected pose.
        """
        print("\nAvailable Poses:")
        for idx, pose_name in enumerate(poses.keys(), start=1):
            print(f"{idx}. {pose_name.replace('_', ' ').title()}")

        try:
            choice = int(input("\nEnter the number corresponding to the desired pose: "))
            if 1 <= choice <= len(poses):
                selected_pose_name = list(poses.keys())[choice - 1]
                return selected_pose_name, poses[selected_pose_name]
            else:
                print("Invalid choice. Please enter a number from the list.")
                return self.select_pose(poses)
        except ValueError:
            print("Invalid input. Please enter a valid number.")
            return self.select_pose(poses)

    def move_to_pose(self, pose):
        """
        Commands the robot to move to the specified pose.
        Returns True if successful, False otherwise.
        """
        self.move_group.set_pose_target(pose, self.end_effector_link)
        rospy.loginfo("Planning motion...")
        success = self.move_group.go(wait=True)

        # Calling stop() ensures that there is no residual movement
        self.move_group.stop()

        # Clear targets after planning
        self.move_group.clear_pose_targets()

        return success

    def execute_pose_movement(self):
        """
        Facilitates user interaction to select and move to desired poses.
        """
        poses = self.define_poses()

        while not rospy.is_shutdown():
            selected_pose_name, selected_pose = self.select_pose(poses)
            rospy.loginfo(f"Selected Pose: {selected_pose_name.replace('_', ' ').title()}")

            rospy.loginfo(f"Moving to '{selected_pose_name.replace('_', ' ').title()}' pose...")
            success = self.move_to_pose(selected_pose)

            if success:
                rospy.loginfo(f"Successfully moved to '{selected_pose_name.replace('_', ' ').title()}' pose.")
            else:
                rospy.logerr(f"Failed to move to '{selected_pose_name.replace('_', ' ').title()}' pose.")

            # Optionally, ask the user if they want to move to another pose
            proceed = input("\nDo you want to move to another pose? (y/n): ").strip().lower()
            if proceed != 'y':
                rospy.loginfo("Exiting Pose Manager.")
                break

    def shutdown(self):
        """
        Cleans up MoveIt Commander.
        """
        moveit_commander.roscpp_shutdown()

def main():
    try:
        pose_manager = PoseManager()
        pose_manager.execute_pose_movement()
        pose_manager.shutdown()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
