#!/usr/bin/env python

import sys, copy, os
import rospy

from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose

from tork_moveit_tutorial import init_node, question_yn, get_ros_version, normalize_orientation


if __name__ == '__main__':
    
    init_node()
    
    group = MoveGroupCommander("botharms")
    
    # Pose Target 1
    pose_target_rarm_1 = Pose()        
    pose_target_rarm_1.position.x = 0.4
    pose_target_rarm_1.position.y = -0.4
    pose_target_rarm_1.position.z = 0.15
    pose_target_rarm_1.orientation.x = 0.0
    pose_target_rarm_1.orientation.y = -0.707
    pose_target_rarm_1.orientation.z = 0.0
    pose_target_rarm_1.orientation.w = 0.707
    
#    ros_version = get_ros_version()
    ros_version = rospy.get_param('/rosdistro')
    ros_version = ros_version.rstrip('\n')
#    ros_version = os.environ["ROS_DISTRO"]
    
    if ros_version == 'kinetic':
        pose_target_rarm_1 = normalize_orientation( pose_target_rarm_1 )
    
    rospy.loginfo( "Right Arm Pose Target 1:\n{}".format( pose_target_rarm_1 ) )
    
    pose_target_larm_1 = Pose()
    pose_target_larm_1.position.x = pose_target_rarm_1.position.x
    pose_target_larm_1.position.y = pose_target_rarm_1.position.y * -1.0
    pose_target_larm_1.position.z = pose_target_rarm_1.position.z
    pose_target_larm_1.orientation.x = pose_target_rarm_1.orientation.x
    pose_target_larm_1.orientation.y = pose_target_rarm_1.orientation.y
    pose_target_larm_1.orientation.z = pose_target_rarm_1.orientation.z
    pose_target_larm_1.orientation.w = pose_target_rarm_1.orientation.w
    
    if ros_version == 'kinetic':
        pose_target_larm_1 = normalize_orientation( pose_target_larm_1 )
    
    rospy.loginfo( "Left Arm Pose Target 1:\n{}".format( pose_target_larm_1 ) )
    
    group.set_pose_target( pose_target_rarm_1, 'RARM_JOINT5_Link' )
    group.set_pose_target( pose_target_larm_1, 'LARM_JOINT5_Link' )
    group.plan()
    group.go()
    
    # Pose Target 2
    pose_target_rarm_2 = Pose()    
    pose_target_rarm_2.position.x = 0.3
    pose_target_rarm_2.position.y = -0.3
    pose_target_rarm_2.position.z = 0.5
    pose_target_rarm_2.orientation.y = -1.0
    
    if ros_version == 'kinetic':
        pose_target_rarm_2 = normalize_orientation( pose_target_rarm_2 )
    
    rospy.loginfo( "Right Arm Pose Target 2:\n{}".format( pose_target_rarm_2 ) )
    
    pose_target_larm_2 = Pose()
    pose_target_larm_2.position.x = pose_target_rarm_2.position.x
    pose_target_larm_2.position.y = pose_target_rarm_2.position.y * -1.0
    pose_target_larm_2.position.z = pose_target_rarm_2.position.z
    pose_target_larm_2.orientation.x = pose_target_rarm_2.orientation.x
    pose_target_larm_2.orientation.y = pose_target_rarm_2.orientation.y
    pose_target_larm_2.orientation.z = pose_target_rarm_2.orientation.z
    pose_target_larm_2.orientation.w = pose_target_rarm_2.orientation.w
    
    if ros_version == 'kinetic':
        pose_target_larm_2 = normalize_orientation( pose_target_larm_2 )
    
    rospy.loginfo( "Left Arm Pose Target 2:\n{}".format( pose_target_larm_2 ) )    
    
    # Move to Pose Target 1
    rospy.loginfo( "Move to Pose Target 1" )    
    group.set_pose_target( pose_target_rarm_1, 'RARM_JOINT5_Link' )
    group.set_pose_target( pose_target_larm_1, 'LARM_JOINT5_Link' )
    group.go()
    
    # Move to Pose Target 2
    rospy.loginfo( "Move to Pose Target 2" )    
    group.set_pose_target( pose_target_rarm_2, 'RARM_JOINT5_Link' )
    group.set_pose_target( pose_target_larm_2, 'LARM_JOINT5_Link' )
    group.go()
    
    # Pose Target 1 & 2 Mixture
    rospy.loginfo( "Move to Pose Target Right:1 Left:2" )    
    group.set_pose_target( pose_target_rarm_1, 'RARM_JOINT5_Link' )
    group.set_pose_target( pose_target_larm_2, 'LARM_JOINT5_Link' )
    group.go()
    
    rospy.loginfo( "Move to Pose Target Right:2 Left:1" )    
    group.set_pose_target( pose_target_rarm_2, 'RARM_JOINT5_Link' )
    group.set_pose_target( pose_target_larm_1, 'LARM_JOINT5_Link' )
    group.go()
    
    # Back to Pose Target 1
    rospy.loginfo( "Go Back to Pose Target 1" )    
    group.set_pose_target( pose_target_rarm_1, 'RARM_JOINT5_Link' )
    group.set_pose_target( pose_target_larm_1, 'LARM_JOINT5_Link' )
    group.go()

