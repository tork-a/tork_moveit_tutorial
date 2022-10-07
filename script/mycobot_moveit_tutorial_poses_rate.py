#!/usr/bin/env python

import sys, copy
import rospy

from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose

from tork_moveit_tutorial import init_node


if __name__ == '__main__':
    
    init_node()
    group = MoveGroupCommander("arm_group")
    
    # Pose Target 1
    pose_target_1 = Pose()
        
    pose_target_1.position.x = 0.15
    pose_target_1.position.y = 0.0
    pose_target_1.position.z = 0.1
    pose_target_1.orientation.x = 0.0
    pose_target_1.orientation.y = -1.0
    pose_target_1.orientation.z = 0.0
    pose_target_1.orientation.w = 0.0
    
    rospy.loginfo( "Start Move Loop / Ctrl-C to Stop \nWaiting 5 seconds" )
    rospy.sleep(5.0)
    
    step = -0.01
    
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        
        rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_1 ) )
        group.set_pose_target( pose_target_1 )
        group.plan()
        group.go()
        
        pose_target_1.position.y += step
        if pose_target_1.position.y < -0.15:
            pose_target_1.position.y = 0.0

        rospy.loginfo( "Waiting Next... / Ctrl-C to Stop" )
        rate.sleep()
    
