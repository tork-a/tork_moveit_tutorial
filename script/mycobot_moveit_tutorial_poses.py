#!/usr/bin/env python

from tork_moveit_tutorial import *
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':

    init_node()

    group = MoveGroupCommander("arm_group")
    # Pose Target 1
    rospy.loginfo( "Start Pose Target 1")
    pose_target_1 = Pose()

    # quaternion_from_euler(0, 0, -1.57079)
    pose_target_1.position.x = 0.3
    pose_target_1.position.y = 0.0
    pose_target_1.position.z = 0.3
    pose_target_1.orientation.x = 0.0
    pose_target_1.orientation.y = 0.0
    pose_target_1.orientation.z = -0.7071
    pose_target_1.orientation.w =  0.7071

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_1 ) )
    group.set_pose_target( pose_target_1 )
    group.go()

    # Pose Target 2
    rospy.loginfo( "Start Pose Target 2")
    pose_target_2 = Pose()

    pose_target_2.position.x = 0.0
    pose_target_2.position.y =-0.3
    pose_target_2.position.z = 0.3
    pose_target_2.orientation.z = -0.7071
    pose_target_2.orientation.w =  0.7071

    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_2 ) )
    group.set_pose_target( pose_target_2 )
    group.go()

    # Compute Cartesian path
    (plan, fraction) = group.compute_cartesian_path([pose_target_1, pose_target_2], 0.01, 0.0)
    group.execute( plan )
