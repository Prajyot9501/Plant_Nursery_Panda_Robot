#!/usr/bin/env python3

# Students: [Add your name]
# Test Script: Basic Panda Movement
# Acknowledgements: Based on MoveIt tutorials

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def main():
    # Initialize moveit_commander and node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_panda_movement', anonymous=True)

    # Initialize robot interface and scene interface
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Initialize move group for the panda arm
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Test 1: Move to a predefined position
    move_group.set_named_target("ready")
    move_group.go(wait=True)
    rospy.sleep(2)

    # Test 2: Move to a specific pose
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.4
    
    move_group.set_pose_target(pose_goal)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    
    rospy.sleep(2)

    # Clean up
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass