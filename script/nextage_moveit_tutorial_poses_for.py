#!/usr/bin/env python

import sys, copy
import rospy

from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose

from tork_moveit_tutorial import init_node, question_yn


if __name__ == '__main__':
    
    init_node()
    group = MoveGroupCommander("right_arm")
    
    # Pose Target 1
    rospy.loginfo( "Start Pose Target 1")
    pose_target_1 = Pose()
        
    pose_target_1.position.x = 0.3
    pose_target_1.position.y = 0.0
    pose_target_1.position.z = 0.2
    pose_target_1.orientation.x = 0.0
    pose_target_1.orientation.y = -0.707
    pose_target_1.orientation.z = 0.0
    pose_target_1.orientation.w = 0.707
    
    step = -0.1
    
    for i in range(5):
        pose_target_1.position.y = i * step
        rospy.loginfo( "Set Target to Pose No.{}:\n{}".format( i, pose_target_1 ) )
    
        group.set_pose_target( pose_target_1 )

        if question_yn( "Start moving to target No.{} ?".format( i ) ):
            group.go()
        else:
            rospy.loginfo( "Skipped Pose No.{}".format( i ) )
    
