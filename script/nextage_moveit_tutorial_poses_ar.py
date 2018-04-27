#!/usr/bin/env python  

import sys, copy, math
import rospy, tf

from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_multiply, quaternion_about_axis

from tork_moveit_tutorial import init_node, get_current_target_pose


def main():

    init_node()
    
    # Preparing Head
    rospy.loginfo( "Preparing Head..." )
    headg = MoveGroupCommander("head")
    headg.set_joint_value_target( [ 0.0, 60.0/180.0*math.pi ] )
    headg.go()
    
    # Preparing Both Arms
    rospy.loginfo( "Preparing Left Arm..." )
    barmg = MoveGroupCommander("botharms")
    barmg.set_pose_target( [ 0.325,  0.482, 0.167, 0.0, -math.pi/2, 0.0 ], 'LARM_JOINT5_Link'  )
    barmg.set_pose_target( [ 0.325, -0.482, 0.167, 0.0, -math.pi/2, 0.0 ], 'RARM_JOINT5_Link'  )
    barmg.go()
    rospy.sleep(2.0)
    
    # Right Arm
    group = MoveGroupCommander("right_arm")
    
    # Frame ID Definitoins
    planning_frame_id = group.get_planning_frame()
    tgt_frame_id = '/ar_marker_4'
    
    # Get a target pose
    pose_target = get_current_target_pose( tgt_frame_id, planning_frame_id, 5.0 )
    
    # Move to a point above target
    if pose_target:
        
        # Rotate Pose for Right Hand
        quat = []
        quat.append( pose_target.pose.orientation.x )
        quat.append( pose_target.pose.orientation.y )
        quat.append( pose_target.pose.orientation.z )
        quat.append( pose_target.pose.orientation.w )
        quat = quaternion_multiply( quat, quaternion_about_axis( math.pi/2, (1,0,0) ) )
        quat = quaternion_multiply( quat, quaternion_about_axis( math.pi/2, (0,0,1) ) )
        pose_target.pose.orientation.x = quat[0]
        pose_target.pose.orientation.y = quat[1]
        pose_target.pose.orientation.z = quat[2]
        pose_target.pose.orientation.w = quat[3]
        
        pose_target.pose.position.z += 0.4
        rospy.loginfo( "Set Target To: \n{}".format( pose_target ) )
        group.set_pose_target( pose_target )
        ret = group.go()
        rospy.loginfo( "Executed ... {}".format( ret ) )        
    else:
        rospy.logwarn( "Pose Error: {}".format( pose_target ) )


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

