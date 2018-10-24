#!/usr/bin/env python

import sys, copy, math
import rospy, tf

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped

from tork_moveit_tutorial import init_node


if __name__ == '__main__':
    
    init_node()
    
    group = MoveGroupCommander("right_arm")
    
    # Pose Target 1
    rospy.loginfo( "Start Pose Target 1")
    pose_target_1 = Pose()
    
    pose_target_1.position.x = 0.3
    pose_target_1.position.y = -0.1
    pose_target_1.position.z = 0.15
    pose_target_1.orientation.x = 0.0
    pose_target_1.orientation.y = -0.707
    pose_target_1.orientation.z = 0.0
    pose_target_1.orientation.w = 0.707
    
    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_1 ) )
    
    group.set_pose_target( pose_target_1 )
    group.go()
    
    # Add Object to Planning Scene
    rospy.loginfo( "Planning Scene Settings")
    scene = PlanningSceneInterface()

    rospy.sleep(2)   # Waiting for PlanningSceneInterface

    box_pose = PoseStamped()
    box_pose.header.frame_id = group.get_planning_frame()
    box_pose.pose.position.x = 0.3
    box_pose.pose.position.y = -0.3
    box_pose.pose.position.z = -0.25
    box_pose.pose.orientation.w = 1.0
    
    scene.add_box( 'box_object', box_pose, ( 0.4, 0.1, 0.5 ) )
    rospy.sleep(2)

    rospy.loginfo( "Scene Objects : {}".format( scene.get_known_object_names() ) )
    
    # Pose Target 2
    rospy.loginfo( "Start Pose Target 2")
    pose_target_2 = Pose()
    pose_target_2.position.x = 0.3
    pose_target_2.position.y = -0.5
    pose_target_2.position.z = 0.15
    pose_target_2.orientation.x = 0.0
    pose_target_2.orientation.y = -0.707
    pose_target_2.orientation.z = 0.0
    pose_target_2.orientation.w = 0.707
    
    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_2 ) )
    
    group.set_pose_target( pose_target_2 )
    group.go()
    
