#!/usr/bin/env python

import rospy

def my_callback(event):
    print( "Timer called at {}".format( event.current_real ) )


if __name__ == '__main__':
    
    rospy.init_node( 'rospy_timing_example', anonymous=True)
    
    # Start Timer
    timer = rospy.Timer( rospy.Duration(2), my_callback )
    
    duration = 20.0
    print( "Stop in {} [sec]".format( duration ) )
    rospy.sleep( duration )

    
