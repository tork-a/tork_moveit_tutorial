#!/usr/bin/env python

import rospy, geometry_msgs.msg, copy

from moveit_commander import MoveGroupCommander

if __name__ == '__main__':
    
    rospy.init_node('commander_example', anonymous=True)
    group = MoveGroupCommander("right_arm")
    
    # Pose Target 1
    pose_target_1 = geometry_msgs.msg.Pose()
        
    pose_target_1.position.x = 0.3
    pose_target_1.position.y = -0.1
    pose_target_1.position.z = 0.15
    pose_target_1.orientation.x = 0.0
    pose_target_1.orientation.y = -0.707
    pose_target_1.orientation.z = 0.0
    pose_target_1.orientation.w = 0.707
    
    # Pose Target 2
    pose_target_2 = geometry_msgs.msg.Pose()
    
    pose_target_2 = copy.deepcopy( pose_target_1 )
    pose_target_2.position.y = -0.5
        
    # Waypoints Motion
    waypoints = []
    waypoints.append( pose_target_1 )
    waypoints.append( pose_target_2 )
    
    ( plan1, fraction ) = group.compute_cartesian_path( waypoints, 0.01, 0.0 )
    rospy.loginfo( "Plan:\n{}".format( plan1 ) )
    group.execute( plan1 )
    
