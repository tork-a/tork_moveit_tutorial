#!/usr/bin/env python

import sys, copy, math
import rospy, tf

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, OrientationConstraint

from tork_moveit_tutorial import init_node


if __name__ == '__main__':
    
    init_node()
    
    group = MoveGroupCommander("right_arm")
    
    
    # Initialize the Planning Scene
    rospy.loginfo( "Setting the Planning Scene...")
    scene = PlanningSceneInterface()
    rospy.sleep(2)

    scene.remove_world_object()    # Remove all objects first
    rospy.sleep(2)
    
    rospy.loginfo( "All objects Removed : {}".format( scene.get_known_object_names() ) )
    
    
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
    
    
    # Add Object to the Planning Scene
    rospy.loginfo( "Add Objects to the Planning Scene..." )
    box_pose = PoseStamped()
    box_pose.header.frame_id = group.get_planning_frame()
    box_pose.pose.position.x = 0.3
    box_pose.pose.position.y = -0.3
    box_pose.pose.position.z = -0.25
    box_pose.pose.orientation.w = 1.0
    
    scene.add_box( 'box_object', box_pose, ( 0.4, 0.1, 0.5 ) )
    rospy.sleep(2)
        
    rospy.loginfo( "Scene Objects : {}".format( scene.get_known_object_names() ) )
    
    
    # Set Path Constraint
    constraints = Constraints()
    constraints.name = "down"
    
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = group.get_planning_frame()
    orientation_constraint.link_name = group.get_end_effector_link()
    orientation_constraint.orientation = pose_target_1.orientation
    orientation_constraint.absolute_x_axis_tolerance = 3.1415
    orientation_constraint.absolute_y_axis_tolerance = 0.05
    orientation_constraint.absolute_z_axis_tolerance = 0.05
    orientation_constraint.weight = 1.0
    
    constraints.orientation_constraints.append( orientation_constraint )
    
    group.set_path_constraints( constraints )
    rospy.loginfo( "Get Path Constraints:\n{}".format( group.get_path_constraints() ) )
    
    
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
    
    group.set_planner_id( "RRTConnectkConfigDefault" )
    group.allow_replanning( True )
    
    rospy.loginfo( "Set Target to Pose:\n{}".format( pose_target_2 ) )
    
    xyz =  [ pose_target_2.position.x,
             pose_target_2.position.y,
             pose_target_2.position.z ]
    group.set_position_target( xyz )
    result_p = group.go()
    
    rospy.loginfo( "Moving to the Position Executed... {}".format( result_p ) )
    
    group.clear_path_constraints()
    
    if result_p:
        group.set_pose_target( pose_target_2 )
        result_o = group.go()
        rospy.loginfo( "Adjusting the Orientation Executed... {}".format( result_o ) )
    
