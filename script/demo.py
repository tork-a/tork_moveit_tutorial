#!/usr/bin/env python  

from tork_moveit_tutorial import *

import rospy, math

import IPython

if __name__ == '__main__':
    
    init_node()
    
    # Waypoints for NEXTAGE OPEN
    waypoints = make_waypoints_example()

    waypoints_mycobot = make_waypoints_example('myCobot')
    
    rpy_nextage = [ 0.0, -math.pi/2, 0.0 ]
    waypoints_circular = make_waypoints_circular( rpy=rpy_nextage )
    waypoints_rectangular = make_waypoints_rectangular( rpy=rpy_nextage )

    rpy_mycobot = [ 0.0, -math.pi, 0.0 ]
    dp_a_mycobot = [ 0.15, 0.15, 0.15 ]
    dp_b_mycobot = [ 0.2, 0.2, 0.15 ]
    center_mycobot = [ 0.18, 0.18, 0.15 ]
    radius_mycobot = 0.03
    waypoints_mycobot_circular = make_waypoints_circular(
                                    center=center_mycobot,
                                    radius=radius_mycobot,
                                    rpy=rpy_mycobot
                                    )
    waypoints_mycobot_rectangular = make_waypoints_rectangular(
                        dp_a=dp_a_mycobot,
                        dp_b=dp_b_mycobot,
                        rpy=rpy_mycobot
                        )
    
    # Start IPython Console
    rospy.loginfo( "[moveit_tutorial_tools] Start IPython Console" )
    IPython.embed()


