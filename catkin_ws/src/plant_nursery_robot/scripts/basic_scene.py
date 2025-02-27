#!/usr/bin/env python3

# Students: [Add your name]
# Basic Scene Setup with STL
# Acknowledgements: MoveIt Tutorials

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import os
import rospkg
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import shape_msgs.msg
import tf.transformations
import geometry_msgs.msg
import math
import random

class BasicScene(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('basic_scene_demo', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm") 
        self.hand_group = moveit_commander.MoveGroupCommander("panda_hand")

        # Add this to disable planning animation
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20
        )
        self.display_trajectory_publisher.unregister()

        # Your existing settings
        self.arm_group.set_max_velocity_scaling_factor(1.0)
        self.arm_group.set_max_acceleration_scaling_factor(1.0)
        self.arm_group.set_planning_time(1.0)
        self.arm_group.set_num_planning_attempts(1)
        self.arm_group.set_goal_orientation_tolerance(0.01)
        self.arm_group.set_goal_position_tolerance(0.01)
        print("Planning Frame:", self.arm_group.get_planning_frame())
        
        # Allow time for scene initialization
        rospy.sleep(2)

    def print_joint_states(self):
        """Print current joint states with names for clarity"""
        joint_values = self.arm_group.get_current_joint_values()
        joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 
                    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        print("\nCurrent Joint States:")
        for name, value in zip(joint_names, joint_values):
            print(f"{name}: {value:.3f} rad = {math.degrees(value):.1f} degrees")


    
    def add_stl_objects(self):
        try:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('plant_nursery_robot')
            
            # Setup planning scene publisher
            self.planning_scene_publisher = rospy.Publisher(
                '/planning_scene',
                moveit_msgs.msg.PlanningScene,
                queue_size=10
            )

            # Add single rectangular table
            table_pose = geometry_msgs.msg.PoseStamped()
            table_pose.header.frame_id = "world"
            table_pose.pose.position.x = 0.5  # Center x position
            table_pose.pose.position.y = 0.0  # Center y position (between pot and watering can)
            table_pose.pose.position.z = 0.0  # Table sits on ground
            table_pose.pose.orientation.w = 1.0
            # Making table longer in y-direction to accommodate both objects
            self.scene.add_box("table", table_pose, size=(0.6, 2.0, 0.4))  # Width(x), Length(y), Height(z)

            # Add a soil bad
            soil_bed_pose = geometry_msgs.msg.PoseStamped()
            soil_bed_pose.header.frame_id = "world"
            soil_bed_pose.pose.position.x = 0.5  # Center x position
            soil_bed_pose.pose.position.y = 0.53  # Center y position (between pot and watering can)
            soil_bed_pose.pose.position.z = 0.2  # Table sits on ground
            soil_bed_pose.pose.orientation.w = 1.0
            # Making table longer in y-direction to accommodate both objects
            self.scene.add_box("soil_bed", soil_bed_pose, size=(0.4, 0.15, 0.05))  # Width(x), Length(y), Height(z)

            # Add soil can cylinder
            soil_can_pose = geometry_msgs.msg.PoseStamped()
            soil_can_pose.header.frame_id = "world"
            soil_can_pose.pose.position.x = 0.4  # Same x as pots
            soil_can_pose.pose.position.y = -0.5  # To the right of pots
            soil_can_pose.pose.position.z = 0.268  # Sits on table
            soil_can_pose.pose.orientation.w = 1.0

            # Add the soil can cylinder
            self.scene.add_cylinder(
                "soil_can",
                soil_can_pose,
                height=0.12,  # Taller than the soil hexagon
                radius=0.035  # Same radius as soil hexagon
            )

            # Add first pot
            pot_path = os.path.join(package_path, 'meshes', 'pot.stl')
            pot1_pose = geometry_msgs.msg.PoseStamped()
            pot1_pose.header.frame_id = "world"
            pot1_pose.pose.position.x = 0.4
            pot1_pose.pose.position.y = -0.25  # Moved back to make room for second pot
            pot1_pose.pose.position.z = 0.201  # Sits on table
            pot1_pose.pose.orientation.w = 1.0
            
            # Add second pot
            pot2_pose = geometry_msgs.msg.PoseStamped()
            pot2_pose.header.frame_id = "world"
            pot2_pose.pose.position.x = 0.6
            pot2_pose.pose.position.y = -0.25  # Placed in front of first pot
            pot2_pose.pose.position.z = 0.201  # Sits on table
            pot2_pose.pose.orientation.w = 1.0
            
            # Add watering can
            can_path = os.path.join(package_path, 'meshes', 'watering_can.stl')
            can_pose = geometry_msgs.msg.PoseStamped()
            can_pose.header.frame_id = "world"
            can_pose.pose.position.x = 0.6
            can_pose.pose.position.y = 0.3
            can_pose.pose.position.z = 0.201 # Sits on table
            can_pose.pose.orientation.w = 1.0

            

            # Add meshes to scene
            self.scene.add_mesh(
                "pot1",  # Changed identifier to distinguish from second pot
                pot1_pose,
                pot_path,
                size=(0.0008, 0.0008, 0.0008)
            )
            
            self.scene.add_mesh(
                "pot2",  # New identifier for second pot
                pot2_pose,
                pot_path,
                size=(0.0008, 0.0008, 0.0008)
            )
            
            self.scene.add_mesh(
                "watering_can",
                can_pose,
                can_path,
                size=(0.0018, 0.0018, 0.0018)
            )

            # Create colors for objects
            table_color = moveit_msgs.msg.ObjectColor()
            table_color.id = "table"
            table_color.color.r = 0.8  # Light brown color
            table_color.color.g = 0.6
            table_color.color.b = 0.4
            table_color.color.a = 1.0

            soil_bed_color = moveit_msgs.msg.ObjectColor()
            soil_bed_color.id = "soil_bed"
            soil_bed_color.color.r = 0.25  # Light brown color
            soil_bed_color.color.g = 0.16
            soil_bed_color.color.b = 0.05
            soil_bed_color.color.a = 1.0

            pot1_color = moveit_msgs.msg.ObjectColor()
            pot1_color.id = "pot1"
            pot1_color.color.r = 0.6
            pot1_color.color.g = 0.3
            pot1_color.color.b = 0.2
            pot1_color.color.a = 1.0

            pot2_color = moveit_msgs.msg.ObjectColor()
            pot2_color.id = "pot2"
            pot2_color.color.r = 0.6
            pot2_color.color.g = 0.3
            pot2_color.color.b = 0.2
            pot2_color.color.a = 1.0

            can_color = moveit_msgs.msg.ObjectColor()
            can_color.id = "watering_can"
            can_color.color.r = 0.3
            can_color.color.g = 0.4
            can_color.color.b = 0.5
            can_color.color.a = 1.0

            soil_can_color = moveit_msgs.msg.ObjectColor()
            soil_can_color.id = "soil_can"
            soil_can_color.color.r = 0.25  # Dark brown color (same as hexagonal soil)
            soil_can_color.color.g = 0.15
            soil_can_color.color.b = 0.05
            soil_can_color.color.a = 1.0

            # Apply colors
            planning_scene = moveit_msgs.msg.PlanningScene()
            planning_scene.is_diff = True
            planning_scene.object_colors.append(table_color)
            planning_scene.object_colors.append(pot1_color)
            planning_scene.object_colors.append(pot2_color)
            planning_scene.object_colors.append(can_color)
            planning_scene.object_colors.append(soil_bed_color)
            planning_scene.object_colors.append(soil_can_color)
            
            rospy.sleep(1)
            self.planning_scene_publisher.publish(planning_scene)
            
            print("Successfully added meshes with colors to scene")
            return True

        except Exception as e:
            print(f"Error loading meshes: {str(e)}")
            return False
                
    def add_tulip(self, position_x, position_y, position_z, name_suffix="1"):
        """
        Adds a complete tulip (stem + head) as a single unit at specified position
        Args:
            position_x: X coordinate for tulip placement
            position_y: Y coordinate for tulip placement
            name_suffix: Identifier for this specific tulip unit
        """
        try:
            # Get package path
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('plant_nursery_robot')
            
            # Define paths for stem and head STL files
            stem_path = os.path.join(package_path, 'meshes', 'stem_min.stl')
            head_path = os.path.join(package_path, 'meshes', 'petals_min.stl')
            
            # Create poses for stem and head
            stem_pose = geometry_msgs.msg.PoseStamped()
            stem_pose.header.frame_id = "world"
            stem_pose.pose.position.x = position_x
            stem_pose.pose.position.y = position_y
            stem_pose.pose.position.z = position_z
            stem_pose.pose.orientation.w = 1.0
            
            head_pose = geometry_msgs.msg.PoseStamped()
            head_pose.header.frame_id = "world"
            head_pose.pose.position.x = position_x - 0.0080
            head_pose.pose.position.y = position_y + 0.0046
            head_pose.pose.position.z = position_z + 0.103  # Adjust height based on your stem height
            head_pose.pose.orientation.w = 1.0
            
            # Add both parts with related names
            tulip_name = f"tulip_{name_suffix}"
            self.scene.add_mesh(
                f"{tulip_name}_stem",
                stem_pose,
                stem_path,
                size=(0.0008, 0.0008, 0.0008)  # Adjust scale as needed
            )
            
            self.scene.add_mesh(
                f"{tulip_name}_head",
                head_pose,
                head_path,
                size=(0.0008, 0.0008, 0.0008)  # Adjust scale as needed
            )
        # Create a publisher for the planning scene
            planning_scene_publisher = rospy.Publisher('/planning_scene', moveit_msgs.msg.PlanningScene, queue_size=10)
            
            # Set the color for the head (petals)
            petal_color = moveit_msgs.msg.ObjectColor()
            petal_color.id = f"{tulip_name}_head"  # Match the exact ID used in add_mesh
            petal_color.color.r = 1.0    # Yellow tulip color
            petal_color.color.g = 0.87   
            petal_color.color.b = 0.41
            petal_color.color.a = 1.0

            # Create and publish the planning scene update
            planning_scene = moveit_msgs.msg.PlanningScene()
            planning_scene.is_diff = True
            planning_scene.object_colors.append(petal_color)
            
            # Give some time for the publisher to initialize
            rospy.sleep(0.5)
            
            # Publish multiple times to ensure the update is received
            for _ in range(3):
                planning_scene_publisher.publish(planning_scene)
                rospy.sleep(0.1)
            
            print(f"Added tulip unit {tulip_name} with yellow petals")
            return tulip_name

        except Exception as e:
            print(f"Error adding tulip: {str(e)}")
            return None
        
    def add_hexagonal_soil(self):
        """Adds a hexagonal 'soil' using a cylinder with 6 sides"""
        try:
            soil_pose = geometry_msgs.msg.PoseStamped()
            soil_pose.header.frame_id = "world"
            soil_pose.pose.position.x = 0.513  # Match pot position
            soil_pose.pose.position.y = 0.008  # Match pot position
            soil_pose.pose.position.z = 0.31  # Adjust height to be near top of pot
            soil_pose.pose.orientation.w = 1.0

            # Add a thin cylinder with 6 sides (appears as hexagon)
            self.scene.add_cylinder(
                "soil",
                soil_pose,
                height=0.030,  # Very thin to look like a soil surface
                radius=0.045   # Match pot's inner radius
            )

            # Create a publisher for the planning scene (if not already created)
            if not hasattr(self, 'planning_scene_publisher'):
                self.planning_scene_publisher = rospy.Publisher(
                    '/planning_scene',
                    moveit_msgs.msg.PlanningScene,
                    queue_size=10
                )

            # Add dark brown color for soil
            soil_color = moveit_msgs.msg.ObjectColor()
            soil_color.id = "soil"
            soil_color.color.r = 0.25  # Dark brown color
            soil_color.color.g = 0.15
            soil_color.color.b = 0.05
            soil_color.color.a = 1.0

            # Apply color
            planning_scene = moveit_msgs.msg.PlanningScene()
            planning_scene.is_diff = True
            planning_scene.object_colors.append(soil_color)
            
            # Give some time for the publisher to initialize
            rospy.sleep(0.5)
            
            # Publish multiple times to ensure the update is received
            for _ in range(3):
                self.planning_scene_publisher.publish(planning_scene)
                rospy.sleep(0.1)

            print("Added hexagonal soil to pot")
            return True

        except Exception as e:
            print(f"Error adding hexagonal soil: {str(e)}")
            return False

    def clear_scene(self):
        # Remove all objects from the scene
        self.scene.remove_world_object()
        rospy.sleep(1)

    def add_tulip_row(self, start_x=0.5, start_y=0.1, num_tulips=3, spacing=0.05):
        """
        Adds a row of tulips with specified spacing
        Args:
            start_x: Starting X coordinate for first tulip
            start_y: Y coordinate for the row
            num_tulips: Number of tulips to add
            spacing: Space between tulips in meters
        """
        tulip_names = []
        for i in range(num_tulips):
            x_pos = start_x + (i * spacing)
            tulip_name = self.add_tulip(
                position_x=x_pos,
                position_y=start_y,
                position_z=0.21,
                name_suffix=str(i+1)
            )
            if tulip_name:
                tulip_names.append(tulip_name)
                rospy.sleep(0.2)  # Small delay between additions
        print(tulip_names)
        return tulip_names

    def open_gripper(self):
        joint_goal = self.hand_group.get_current_joint_values()
        joint_goal[0] = 0.04  # Opening width
        joint_goal[1] = 0.04  # Symmetric opening
        self.hand_group.go(joint_goal, wait=True)
        self.hand_group.stop()
        rospy.sleep(1)

    def close_gripper(self):
        joint_goal = self.hand_group.get_current_joint_values()
        joint_goal[0] = 0.013 # Closing width
        joint_goal[1] = 0.013  # Symmetric closing
        self.hand_group.go(joint_goal, wait=True)
        self.hand_group.stop()
        rospy.sleep(1)

    def wait_for_state_update(self, object_name, object_is_known=False, object_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the object is in attached objects
            attached_objects = self.scene.get_attached_objects([object_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the object is in the scene
            is_known = object_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (object_is_attached == is_attached) and (object_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def attach_object(self, object_name):
        touch_links = self.robot.get_link_names(group="panda_hand")
        self.scene.attach_box("panda_hand", object_name, touch_links=touch_links)
        return self.wait_for_state_update(object_name, object_is_attached=True, object_is_known=False)

    def detach_object(self, object_name):
        self.scene.remove_attached_object("panda_hand", name=object_name)
        return self.wait_for_state_update(object_name, object_is_known=True, object_is_attached=False)
    
    def move_to_joint_angles(self, j1, j2, j3, j4, j5, j6, j7):
        """
        Move robot to specified joint angles (in degrees)
        Args:
            j1-j7: Joint angles in degrees for joints 1-7
        Returns:
            success: Boolean indicating if movement was successful
        """
        # Convert degrees to radians
        joint_goal = [
            math.radians(j1),  # Joint 1
            math.radians(j2),  # Joint 2 
            math.radians(j3),  # Joint 3
            math.radians(j4),  # Joint 4
            math.radians(j5),  # Joint 5
            math.radians(j6),  # Joint 6
            math.radians(j7)   # Joint 7
        ]
        
        # Set and execute the joint goal
        self.arm_group.set_joint_value_target(joint_goal)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        rospy.sleep(1)
        
        # if success:
        #     print("Successfully moved to joint angles")
        # else:
        #     print("Failed to move to joint angles")
        
        return success
    
  
        # Start from ready position
        self.arm_group.set_named_target("ready")
        self.arm_group.go(wait=True)
        rospy.sleep(1)
        
        # Set explicit joint values for the desired position and orientation
        joint_goal = [
            0.0,            # panda_joint1 (base)
            -0.785,         # panda_joint2 (-45 degrees)
            0.0,            # panda_joint3
            -2.356,         # panda_joint4 (-135 degrees)
            0.0,            # panda_joint5
            1.571,          # panda_joint6 (90 degrees)
            0.0             # panda_joint7 (set to 0 for flat gripper)
        ]
        
        self.arm_group.set_joint_value_target(joint_goal)
        self.arm_group.go(wait=True)
        rospy.sleep(1)
        
        # Now try to move to the desired position while maintaining orientation
        current_pose = self.arm_group.get_current_pose().pose
        target_pose = geometry_msgs.msg.Pose()
        
        # Keep the current orientation
        target_pose.orientation = current_pose.orientation
        
        # Set new position
        target_pose.position.x = 0.3
        target_pose.position.y = 0.3
        target_pose.position.z = 0.5
        
        # Move to target while trying to maintain orientation
        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def attach_tulip_parts(self, tulip_name):
        """
        Attach both stem and head of a tulip using existing attach_object function
        Args:
            tulip_name: Base name of the tulip (e.g., "tulip_1")
        """
        # Attach stem
        stem_success = self.attach_object(f"{tulip_name}_stem")
        # Attach head
        head_success = self.attach_object(f"{tulip_name}_head")
        
        return stem_success and head_success

    def detach_tulip_parts(self, tulip_name):
        """
        Detach both stem and head using existing detach_object function
        """
        # Detach stem
        stem_success = self.detach_object(f"{tulip_name}_stem")
        # Detach head
        head_success = self.detach_object(f"{tulip_name}_head")
        
        return stem_success and head_success

    def add_moisture_sensor(self, position_x, position_y, position_z, name="moisture_sensor"):
        """Adds a visual moisture sensor (represented as a small cylinder)"""
        try:
            sensor_pose = geometry_msgs.msg.PoseStamped()
            sensor_pose.header.frame_id = "world"
            sensor_pose.pose.position.x = position_x
            sensor_pose.pose.position.y = position_y
            sensor_pose.pose.position.z = position_z
            sensor_pose.pose.orientation.w = 1.0

            # Add a small cylinder to represent the sensor
            self.scene.add_cylinder(
                name,
                sensor_pose,
                height=0.02,  # Very thin
                radius=0.01   # Small radius
            )

            # Create a publisher for the planning scene
            if not hasattr(self, 'planning_scene_publisher'):
                self.planning_scene_publisher = rospy.Publisher(
                    '/planning_scene',
                    moveit_msgs.msg.PlanningScene,
                    queue_size=10
                )

            # Default color (grey - inactive)
            self.update_moisture_sensor_color(0.5, name)  # 0.5 = neutral moisture
            return True

        except Exception as e:
            print(f"Error adding moisture sensor: {str(e)}")
            return False

    def update_moisture_sensor_color(self, moisture_level, sensor_name="moisture_sensor"):
        """
        Updates sensor color based on moisture level (0.0 to 1.0)
        0.0 = very dry (red)
        0.5 = moderate (yellow)
        1.0 = well watered (green)
        """
        # Calculate color based on moisture level
        if moisture_level < 0.3:
            r, g, b = 0.9, 0.2, 0.2  # Red
        elif moisture_level < 0.7:
            r, g, b = 0.9, 0.9, 0.2  # Yellow
        else:
            r, g, b = 0.2, 0.8, 0.2  # Green

        sensor_color = moveit_msgs.msg.ObjectColor()
        sensor_color.id = sensor_name
        sensor_color.color.r = r
        sensor_color.color.g = g
        sensor_color.color.b = b
        sensor_color.color.a = 1.0

        # Apply color
        planning_scene = moveit_msgs.msg.PlanningScene()
        planning_scene.is_diff = True
        planning_scene.object_colors.append(sensor_color)
        
        self.planning_scene_publisher.publish(planning_scene)
        rospy.sleep(0.1)

    def check_soil_moisture(self):
        """
        Simulates checking soil moisture with the robot
        Returns moisture level (0.0 to 1.0)
        """
        try:
            print("\nChecking soil moisture...")
            
            # Move to position above sensor
            self.move_to_joint_angles(75, -73, -55, -141, -134, 138, -54)
            rospy.sleep(0.5)
            
            # Move down to "touch" sensor
            self.move_to_joint_angles(74, -79, -73, -151, -161, 135, -41)
            rospy.sleep(1)
            
            # Simulate moisture reading (random for demo)
            current_moisture = random.uniform(0.0, 1.0)
            
            # Update sensor color
            self.update_moisture_sensor_color(current_moisture)
            
            # Print moisture status
            if current_moisture < 0.3:
                print("Soil is very dry! Watering needed.")
            elif current_moisture < 0.7:
                print("Soil moisture is moderate.")
            else:
                print("Soil is well watered.")
                
            return current_moisture
            
        except Exception as e:
            print(f"Error checking moisture: {str(e)}")
            return None

    def task1_put_pot_in_middle(self):
        try:
            print("\nExecuting Task 1: Put Pot in Middle")
            
            # Move to home position first
            self.arm_group.set_named_target("ready")
            self.arm_group.go(wait=True)
            rospy.sleep(1)

            # Lifting the 1st pot from side
            print("Picking up the first pot")
            self.move_to_joint_angles(23, -41, -85, -150, -114, 197, -88)

            # Attach object in scene
            print("Attaching pot...")
            self.attach_object("pot1")
            rospy.sleep(1)

            # Putting the first pot in the main space
            print("Putting the pot down")
            self.move_to_joint_angles(57, -54, -64, -156, -128, 150, -45)

            # Detach object
            print("Detaching pot...")
            self.detach_object("pot1")

            return True
        except Exception as e:
            print(f"Error in Task 1: {str(e)}")
            return False

    def task2_add_soil(self):
        try:
            print("\nExecuting Task 2: Add Soil")
            
            # Picking up the soil can
            print("Picking up soil can")
            self.move_to_joint_angles(47, -70, -99, -124, 52, 163, 94)

            print("Attaching soil can...")
            self.attach_object("soil_can")

            # Inter. pose for above the can
            self.move_to_joint_angles(-164, 58, 146, -155, 166, 175, 32)

            self.move_to_joint_angles(-166, 57, 149, -155, 162, 174, -88)

            self.add_hexagonal_soil()
            self.add_moisture_sensor(0.513, 0.008, 0.32)  # Position near the soil surface

            self.move_to_joint_angles(-164, 58, 146, -155, 166, 175, 32)

            # Picking up the soil can
            print("Putting down soil can")
            self.move_to_joint_angles(47, -70, -99, -124, 52, 163, 94)

            print("Detaching soil can...")
            self.detach_object("soil_can")

            return True
        except Exception as e:
            print(f"Error in Task 2: {str(e)}")
            return False

    def task3_water_soil(self):
        try:
            print("\nExecuting Task 3: Water Soil")
        
            # Check moisture first
            moisture_level = self.check_soil_moisture()
            
            if moisture_level is None:
                print("Error checking moisture! Proceeding with watering anyway...")
            elif moisture_level >= 0.7:
                print("Soil is already well watered. Skipping watering.")
                return True
            
            # Move above watering can handle
            print("Moving above watering can...")
            self.move_to_joint_angles(-64, -61, 74, -118, 58, 90, 41)

            # Attach object in scene
            print("Attaching watering can...")
            self.attach_object("watering_can")
            rospy.sleep(1)

            print("Lifting the watering can up")
            self.move_to_joint_angles(-52, -58, 47, -101, 45, 61, 49) 

            print("Moving above the pot")
            self.move_to_joint_angles(-59, -55, 25, -114, 23, 64, 10) 

            print("Watering")
            self.move_to_joint_angles(-66, -28, 49, -94, 0, 33, 16) 

            print("Finishing")
            self.move_to_joint_angles(-59, -55, 25, -114, 23, 64, 10)  

            print("Going back")
            self.move_to_joint_angles(-68, -53, 64, -99, 48, 75, 46) 

            print("Putting down watering can...")
            self.move_to_joint_angles(-64, -61, 74, -118, 58, 90, 41) 

            # Detach object
            print("Detaching watering can...")
            self.detach_object("watering_can")

            self.update_moisture_sensor_color(0.9)  # Show well-watered state
            print("Soil is now well watered!")
            
            return True

            return True
        except Exception as e:
            print(f"Error in Task 3: {str(e)}")
            return False

    def task4_add_tulip(self):
        try:
            print("\nExecuting Task 4: Add Tulip")
            
            # Tulip in pot location
            self.move_to_joint_angles(41, 22, 30 ,-119, -114, 97, 4)

            # Close gripper
            print("Closing gripper...")
            self.close_gripper()

            self.attach_tulip_parts("tulip_1")

            self.move_to_joint_angles(36, 0, 31, -107, -97, 114, -26)

            self.move_to_joint_angles(32, 10, 27, -95, -103, 121, -27)

            self.move_to_joint_angles(75, -73, -55, -141, -134, 138, -54)

            self.move_to_joint_angles(74, -79, -73, -151, -161, 135, -41)

            print("Opening gripper...")
            self.open_gripper()

            self.detach_tulip_parts("tulip_1")

            return True
        except Exception as e:
            print(f"Error in Task 4: {str(e)}")
            return False

    def task6_water_soil_bed(self):
        try:
            print("\nExecuting Task 6: Water Soil Bed")
            
            # Move above watering can handle
            print("Moving above watering can...")
            self.move_to_joint_angles(-64, -61, 74, -118, 58, 90, 41)

            # Close gripper
            print("Closing gripper...")
            self.close_gripper()

            # Attach object in scene
            print("Attaching watering can...")
            self.attach_object("watering_can")
            rospy.sleep(1)

            print("Lifting the watering can up")
            self.move_to_joint_angles(-52, -58, 47, -101, 45, 61, 49) 

            print("start of soil bed")
            self.move_to_joint_angles(-71, -52, 71, -63, 62, 58, 79) 

            print("Watering")
            self.move_to_joint_angles(-109, -46, 105, -33, 89, 37, 105) 

            print("start of soil bed")
            self.move_to_joint_angles(-71, -52, 71, -63, 62, 58, 79)

            print("Lifting the watering can up")
            self.move_to_joint_angles(-52, -58, 47, -101, 45, 61, 49) 

            print("Putting down watering can...")
            self.move_to_joint_angles(-64, -61, 74, -118, 58, 90, 41)

            print("Detaching watering can...")
            self.detach_object("watering_can")

            # Open gripper
            print("Opening gripper...")
            self.open_gripper()

            return True
        except Exception as e:
            print(f"Error in Task 6: {str(e)}")
            return False

    def run_complete_sequence(self):
        """Run all tasks in sequence"""
        try:
            print("\nRunning complete sequence...")
            
            tasks = [
                self.task1_put_pot_in_middle,
                self.task2_add_soil,
                self.task3_water_soil,
                self.task4_add_tulip,
                self.task3_water_soil,  # Task 5 is same as task 3
                self.task6_water_soil_bed
            ]
            
            for i, task in enumerate(tasks, 1):
                print(f"\nExecuting Step {i} of {len(tasks)}...")
                if not task():
                    print(f"Sequence failed at step {i}")
                    return False
                rospy.sleep(1)
            
            return True
        except Exception as e:
            print(f"Error in sequence: {str(e)}")
            return False
def main():
    try:
        print("Initializing Basic Scene Demo")
        scene_demo = BasicScene()
        
        print("Clearing the scene...")
        scene_demo.clear_scene()
        
        print("Adding STL object to the scene...")
        scene_demo.add_stl_objects()
        

        print("Adding row of tulips to the scene...")
        scene_demo.add_tulip_row(
            start_x=0.3,
            start_y=0.445,
            num_tulips=4,
            spacing=0.075
        )

        # Track completed tasks
        completed_tasks = {
            1: False,  # Put Pot in Middle
            2: False,  # Add Soil
            3: False,  # Water Soil
            4: False,  # Add Tulip
            5: False,  # Water Again
        }

        while True:
            print("\nPlant Nursery Robot Control Menu")
            print("================================")
            print("1. Put Pot in Middle")
            print("2. Add Soil")
            print("3. Water Soil")
            print("4. Add Tulip")
            print("5. Water Again")
            print("6. Check Soil Moisture (Independent Task)")
            print("7. Water Soil Bed (Independent Task)")
            print("8. Run Complete Sequence")
            print("9. Exit")
            
            choice = input("\nEnter your choice (1-9): ")
            
            if choice == '9':
                print("Exiting program...")
                break
                
            elif choice == '8':
                scene_demo.run_complete_sequence()
                # Mark all tasks as completed
                for task in completed_tasks:
                    completed_tasks[task] = True
                    
            elif choice == '7':
                # Independent task, no prerequisites
                scene_demo.task6_water_soil_bed()

            elif choice == '6':
                # Independent task, no prerequisites
                scene_demo.check_soil_moisture()
                
            elif choice in '12345':
                task_num = int(choice)
                
                # Check prerequisites except for task 1
                if task_num > 1:
                    if not completed_tasks[task_num - 1]:
                        print(f"\nError: You must complete Task {task_num - 1} first!")
                        continue
                
                # Execute the chosen task
                success = False
                if task_num == 1:
                    success = scene_demo.task1_put_pot_in_middle()
                elif task_num == 2:
                    success = scene_demo.task2_add_soil()
                elif task_num == 3:
                    success = scene_demo.task3_water_soil()
                elif task_num == 4:
                    success = scene_demo.task4_add_tulip()
                elif task_num == 5:
                    success = scene_demo.task3_water_soil()  # Same as task 3
                
                if success:
                    completed_tasks[task_num] = True
                    print(f"\nTask {task_num} completed successfully!")
                else:
                    print(f"\nTask {task_num} failed!")
            
            else:
                print("\nInvalid choice! Please enter a number between 1 and 8.")
            
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()