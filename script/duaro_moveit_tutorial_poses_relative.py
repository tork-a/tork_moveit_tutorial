#!/usr/bin/env python

import sys, copy
import rospy

from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped

from tork_moveit_tutorial import init_node


if __name__ == '__main__':
    
    init_node()
    
    # Preparing Lower Arm
    rospy.loginfo( "Preparing Lower Arm..." )
    larmg = MoveGroupCommander("lower_arm")
    larm_init_pose = Pose()
    larm_init_pose.position.x = 0.1
    larm_init_pose.position.y = 0.5
    larm_init_pose.position.z = 1.0
    larm_init_pose.orientation.x = 0.0
    larm_init_pose.orientation.y = 0.0
    larm_init_pose.orientation.z = 0.707
    larm_init_pose.orientation.w = 0.707
    larmg.set_pose_target(larm_init_pose)
    larmg.go()
    
    
    # Upper Arm
    group = MoveGroupCommander("upper_arm")
    
    initial_reference_frame = group.get_pose_reference_frame()
    rospy.loginfo( "Initial Reference Frame: {}".format( initial_reference_frame ) )
    
    initial_pose = group.get_current_pose()
    rospy.loginfo( "Initial Pose:\n{}".format( initial_pose ) )
    
    # Relative Target Pose
    target_pose = Pose()
    target_pose.position.x = 0.2
    target_pose.orientation.w = 1.0
    
    # Relative Target with PoseStamped
    rospy.loginfo( "Using PoseStamped" )
    
    target_posestamped = PoseStamped()
    target_posestamped.pose.position.x = 0.2
    target_posestamped.pose.orientation.w = 1.0
    target_posestamped.header.frame_id = '/lower_link_j4'
    target_posestamped.header.stamp = rospy.Time.now()
    
    rospy.loginfo( "Target Pose:\n{}".format( target_posestamped ) )
    group.set_pose_target( target_posestamped )
    group.go()
    
    rospy.loginfo( "Current Reference Frame: {}".format( group.get_pose_reference_frame() ) )
    
    # Go Back to Initial Pose
    group.set_pose_target( initial_pose.pose )
    group.go()
        
    # Relative Target with set_pose_reference_frame
    rospy.loginfo( "Using set_pose_refercence_frame() and Pose" )
    
    group.set_pose_reference_frame( '/lower_link_j4' )
    rospy.loginfo( "Current Reference Frame: {}".format( group.get_pose_reference_frame() ) )
    rospy.loginfo( "Target Pose:\n{}".format( target_pose ) )
    
    group.set_pose_target( target_pose )
    group.go()

    # Reset Pose Reference Frame
    group.set_pose_reference_frame( initial_reference_frame )
    rospy.loginfo( "Current Reference Frame: {}".format( group.get_pose_reference_frame() ) )
    
    # Go Back to Initial Pose
    rospy.loginfo( "Go Back to Initial Pose..." )
    group.set_pose_target( initial_pose )
    group.go()
    
