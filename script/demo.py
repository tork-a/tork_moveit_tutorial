#!/usr/bin/env python  

from tork_moveit_tutorial import *

import rospy, math

import IPython

if __name__ == '__main__':
    
    init_node()
    
    # Waypoints for NEXTAGE OPEN
    waypoints = make_waypoints_example()
    
    rpy_nextage = [ 0.0, -math.pi/2, 0.0 ]
    waypoints_circular = make_waypoints_circular( rpy=rpy_nextage )
    waypoints_rectangular = make_waypoints_rectangular( rpy=rpy_nextage )
    
    # Start IPython Console
    rospy.loginfo( "[moveit_tutorial_tools] Start IPython Console" )
    IPython.embed()


