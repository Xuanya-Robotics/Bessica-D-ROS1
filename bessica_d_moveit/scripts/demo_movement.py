#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
import tf2_ros
import geometry_msgs.msg
import numpy as np

class BessicaDMotion:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('bessica_d_motion', anonymous=True)

        self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
        self.both_arms_group = moveit_commander.MoveGroupCommander("both_arms")

    def move_to_joint_state(self, joint_state):
        """
        Moves the robot to a specified joint state for both arms.
        :param joint_state: A dictionary where keys are joint names and 
                            values are the target joint positions.
        """
        self.both_arms_group.set_joint_value_target(joint_state)
        self.both_arms_group.go(wait=True)

    def move_left_arm(self, joint_positions):
        self.left_arm_group.set_joint_value_target(joint_positions)
        self.left_arm_group.go(wait=True)

    def move_right_arm(self, joint_positions):
        self.right_arm_group.set_joint_value_target(joint_positions)
        self.right_arm_group.go(wait=True)

    def move_both_arms(self, left_joint_positions, right_joint_positions):
        # Note: This method is less direct than move_to_joint_state.
        # It sets targets for individual arm groups, then plans for the combined group.
        # The new `move_to_joint_state` is recommended when you have a full robot state dictionary.
        
        # Create a dictionary to hold the target positions for both arms
        target_joint_values = {}
        left_joint_names = self.left_arm_group.get_active_joints()
        right_joint_names = self.right_arm_group.get_active_joints()

        for i, name in enumerate(left_joint_names):
            if i < len(left_joint_positions):
                target_joint_values[name] = left_joint_positions[i]
        
        for i, name in enumerate(right_joint_names):
            if i < len(right_joint_positions):
                target_joint_values[name] = right_joint_positions[i]

        self.both_arms_group.set_joint_value_target(target_joint_values)
        self.both_arms_group.go(wait=True)
    
    def get_robot_state(self):
        # This method's name might be misleading. It gets the MoveIt! state object, not just joint values.
        left_state = self.left_arm_group.get_current_state()
        right_state = self.right_arm_group.get_current_state()
        return left_state, right_state

    def get_left_arm_joint_values(self):
        return self.left_arm_group.get_current_joint_values()
    
    def get_right_arm_joint_values(self):
        return self.right_arm_group.get_current_joint_values()
    
    def get_both_arms_joint_values(self):
        left_joint_values = self.left_arm_group.get_current_joint_values()
        right_joint_values = self.right_arm_group.get_current_joint_values()
        return left_joint_values, right_joint_values