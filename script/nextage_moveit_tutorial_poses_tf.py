#!/usr/bin/env python  

import sys, copy, math
import rospy

from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose

from tork_moveit_tutorial import init_node, get_current_target_pose


def main():

    init_node()
    
    # Preparing Left Arm
    rospy.loginfo( "Preparing Left Arm..." )
    larmg = MoveGroupCommander("left_arm")
    larmg.set_pose_target( [ 0.325, 0.182, 0.067, 0.0, -math.pi/2, 0.0 ] )
    larmg.go()
    
    
    # Right Arm
    group = MoveGroupCommander("right_arm")
    
    # Frame ID Definitoins
    planning_frame_id = group.get_planning_frame()
    tgt_frame_id = '/LARM_JOINT5_Link'
    
    # Get a target pose
    pose_target = get_current_target_pose( tgt_frame_id, planning_frame_id )
    
    # Move to a point above target
    if pose_target:
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

