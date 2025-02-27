#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

class PlantNurseryRobot:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('plant_nursery_robot', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(2)  # Give the scene time to initialize

        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.move_group_hand = moveit_commander.MoveGroupCommander("panda_hand")
        
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20)

        # Initialize workspace
        self.setup_workspace()
        
        # Tool positions
        self.tool_positions = {
            "watering_can": {"x": 0.4, "y": -0.3, "z": 0.25},
            "soil_scoop": {"x": 0.4, "y": -0.4, "z": 0.25},
            "nutrient_dispenser": {"x": 0.4, "y": -0.5, "z": 0.25}
        }

    def setup_workspace(self):
        # Clear the scene first
        self.scene.remove_world_object()
        rospy.sleep(1)

        # Add tables
        self.add_box("main_table", 0.5, 0.0, 0.0, 1.0, 1.5, 0.05)
        self.add_box("tool_table", 0.4, -0.4, 0.0, 0.3, 0.8, 0.05)

        # Add pots
        self.add_cylinder("pot1", 0.5, 0.2, 0.1, 0.05, 0.1)
        self.add_cylinder("pot2", 0.5, -0.2, 0.1, 0.05, 0.1)
        
        # Add tools
        self.add_box("watering_can", 0.4, -0.3, 0.1, 0.1, 0.1, 0.2)
        self.add_cylinder("soil_scoop", 0.4, -0.4, 0.1, 0.05, 0.15)
        self.add_box("nutrient_dispenser", 0.4, -0.5, 0.1, 0.08, 0.08, 0.15)

        pot_pose = geometry_msgs.msg.PoseStamped()
        pot_pose.header.frame_id = "world"
        pot_pose.pose.position.x = 0.5
        pot_pose.pose.position.y = 0.0
        pot_pose.pose.position.z = 0.25
        pot_pose.pose.orientation.w = 1.0
    
        self.scene.add_mesh("realistic_pot", pot_pose, 
                       "package://plant_nursery_robot/meshes/pot.stl", 
                       (1, 1, 1))  # scale

    def add_box(self, name, x, y, z, size_x, size_y, size_z):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        box_pose.pose.orientation.w = 1.0
        self.scene.add_box(name, box_pose, size=(size_x, size_y, size_z))

    def add_cylinder(self, name, x, y, z, radius, height):
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = "world"
        cylinder_pose.pose.position.x = x
        cylinder_pose.pose.position.y = y
        cylinder_pose.pose.position.z = z
        cylinder_pose.pose.orientation.w = 1.0
        self.scene.add_cylinder(name, cylinder_pose, height, radius)

    def move_to_pose(self, x, y, z, roll=0, pitch=pi, yaw=0):
        pose_goal = geometry_msgs.msg.Pose()
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        self.move_group.set_pose_target(pose_goal)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def move_gripper(self, open_width):
        joint_goal = self.move_group_hand.get_current_joint_values()
        joint_goal[0] = open_width
        joint_goal[1] = open_width
        self.move_group_hand.go(joint_goal, wait=True)
        self.move_group_hand.stop()

    def pick_object(self, obj_name, x, y, z):
        # Move above object
        self.move_to_pose(x, y, z + 0.2)
        # Open gripper
        self.move_gripper(0.08)
        # Move down to object
        self.move_to_pose(x, y, z + 0.05)
        # Close gripper
        self.move_gripper(0.02)
        # Move up
        self.move_to_pose(x, y, z + 0.2)

    def place_object(self, x, y, z):
        # Move to place position
        self.move_to_pose(x, y, z + 0.2)
        # Move down
        self.move_to_pose(x, y, z + 0.05)
        # Open gripper
        self.move_gripper(0.08)
        # Move up
        self.move_to_pose(x, y, z + 0.2)

    def pour_water(self, pot_name):
        # Get pot position (you'll need to store these)
        if pot_name == "pot1":
            pot_x, pot_y = 0.5, 0.2
        else:
            pot_x, pot_y = 0.5, -0.2

        # Pick up watering can
        self.pick_object("watering_can", **self.tool_positions["watering_can"])
        
        # Move to pouring position
        self.move_to_pose(pot_x, pot_y, 0.3, roll=0, pitch=pi/4, yaw=0)
        rospy.sleep(2)  # Simulate pouring
        
        # Return watering can
        self.place_object(**self.tool_positions["watering_can"])

    def run_user_interface(self):
        while not rospy.is_shutdown():
            print("\nPlant Nursery Robot Control")
            print("1. Move pot")
            print("2. Water plant")
            print("3. Add soil")
            print("4. Add nutrients")
            print("5. Exit")
            
            choice = input("Enter your choice (1-5): ")
            
            if choice == "1":
                pot = input("Which pot? (pot1/pot2): ")
                x = float(input("Target X position: "))
                y = float(input("Target Y position: "))
                if pot in ["pot1", "pot2"]:
                    self.pick_object(pot, 0.5, 0.2 if pot == "pot1" else -0.2, 0.1)
                    self.place_object(x, y, 0.1)
            elif choice == "2":
                pot = input("Which pot to water? (pot1/pot2): ")
                self.pour_water(pot)
            elif choice == "5":
                break
            else:
                print("Invalid choice. Please try again.")

def main():
    try:
        robot = PlantNurseryRobot()
        print("Robot initialized. Starting user interface...")
        robot.run_user_interface()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()