#!/usr/bin/env python  

## workaround until https://github.com/ros-planning/moveit/pull/581 is released
import sys
sys.modules["pyassimp"] = sys
import pyassimp
   
import sys, math, copy, subprocess, numpy
import rospy, tf, geometry_msgs.msg, rospkg

from distutils.version import LooseVersion
import python_qt_binding
if LooseVersion(python_qt_binding.QT_BINDING_VERSION).version[0] >= 5:
    from python_qt_binding.QtWidgets import QApplication, QMessageBox
else:
    from python_qt_binding.QtGui import QApplication, QMessageBox

from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose, PoseStamped

def init_node( node_name = "commander_example" ):
    '''
    Initializing QtApplication and ROS node.
    Displaying names of Move Groups defined in the robot.
    
    @type  node_name : str
    @param node_name : name for ROS node to initialize
    '''
    
    global qtapp
    qtapp = QApplication(sys.argv)
    rospy.init_node( node_name, anonymous=True )
    
    robot = RobotCommander()
    
    group_names = robot.get_group_names()
    gnames_info = "Move Groups defined in the robot :"
    for name in group_names:
        gnames_info = gnames_info + '\n' + name
    rospy.loginfo( gnames_info )


def question_yn( qmsg='Message', title='Question' ):
    '''
    Asking Yes or No using PyQt QMessgageBox()
    Return 'True' only in the case that 'Yes' is chosen.
    
    @type  qmsg  : str
    @param qmsg  : Question message for Yes/No answer (Default='Message')
    @type  title : str
    @param title : Title of the message box window (Default='Question')
    
    @rtype  : bool
    @return : Return when 'Yes' is chosen.
    '''
    
    msgbox = QMessageBox()
    result = msgbox.question( msgbox, title, qmsg, msgbox.Yes | msgbox.No, msgbox.No )
    
    if result == msgbox.Yes:
        return True
    else:
        return False


def get_current_target_pose( target_frame_id, base_frame_id, timeout = 1.0 ):
    '''
    Get current pose TF between a target frame and a base frame. 
    
    @type  target_frame_id : str
    @param target_frame_id : Target frame ID for aquiring TF
    @type  base_frame_id   : str
    @param base_frame_id   : Base frame ID for aquiring TF
    @type  timeout         : float
    @param timeout         : Time length for TF translation timeout [s]
    
    @rtype  : PoseStamped
    @return : tf result as PoseStamped
    '''
    
    endtime = rospy.get_time()
    rospy.loginfo( "Waiting Clock: {}".format( endtime ) )
    while not endtime:
        endtime = rospy.get_time()
    
    endtime += timeout
    
    target_pose = None
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            now = rospy.Time(0)
            (trans,quat) = listener.lookupTransform( base_frame_id, target_frame_id, now )
            target_pose = PoseStamped()
            target_pose.pose.position.x = trans[0]
            target_pose.pose.position.y = trans[1]
            target_pose.pose.position.z = trans[2]
            target_pose.pose.orientation.x = quat[0]
            target_pose.pose.orientation.y = quat[1]
            target_pose.pose.orientation.z = quat[2]
            target_pose.pose.orientation.w = quat[3]
            target_pose.header.frame_id = base_frame_id
            target_pose.header.stamp = now
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(e)
        
        now_float = rospy.get_time()
        if endtime < now_float:
            rospy.logwarn( "Time Out: {} [sec] at Clock: {} [sec]".format( timeout, now_float ) )
            break
        
        rate.sleep()

    return target_pose


def make_waypoints_example( rtype="NEXTAGE" ):
    '''
    Making waypoints example data for the each robot.
    
    @type  target_frame_id : str
    @param target_frame_id : Robot Type ("NEXTAGE"/...TBD)
    
    @rtype  : [ Pose, Pose, ... ]
    @return : Array of Pose waypoints
    '''

    wpts = []
    if rtype == "NEXTAGE":
        # 1st Pose
        pose_target = Pose()
        pose_target.position.x = 0.4
        pose_target.position.y = -0.4
        pose_target.position.z = 0.15
        pose_target.orientation.x = 0.0
        pose_target.orientation.y = -0.707
        pose_target.orientation.z = 0.0
        pose_target.orientation.w = 0.707
        wpts.append( copy.deepcopy( pose_target ) )
        
        # 2nd Pose
        pose_target.position.y = -0.2
        wpts.append( copy.deepcopy( pose_target ) )
        
        # 3rd Pose
        pose_target.position.x = 0.3
        pose_target.position.y = -0.3
        pose_target.position.z = 0.5
        pose_target.orientation.x = 0.0
        pose_target.orientation.y = -1.0
        pose_target.orientation.z = 0.0
        pose_target.orientation.w = 0.0
        wpts.append( copy.deepcopy( pose_target ) )
        
        # 4th Pose
        pose_target.position.y = -0.5
        wpts.append( copy.deepcopy( pose_target ) )
    elif rtype == "myCobot":
        # 1st Pose
        pose_target = Pose()
        pose_target.position.x = 0.1
        pose_target.position.y = -0.1
        pose_target.position.z = 0.1
        pose_target.orientation.x = 0.0
        pose_target.orientation.y = -0.707
        pose_target.orientation.z = 0.0
        pose_target.orientation.w = 0.707
        wpts.append( copy.deepcopy( pose_target ) )
        
        # 2nd Pose
        pose_target.position.y = -0.15
        wpts.append( copy.deepcopy( pose_target ) )
        
        # 3rd Pose
        pose_target.position.x = 0.1
        pose_target.position.y = -0.1
        pose_target.position.z = 0.3
        pose_target.orientation.x = 0.0
        pose_target.orientation.y = -1.0
        pose_target.orientation.z = 0.0
        pose_target.orientation.w = 0.0
        wpts.append( copy.deepcopy( pose_target ) )
        
        # 4th Pose
        pose_target.position.y = -0.2
        wpts.append( copy.deepcopy( pose_target ) )
        
    return wpts


def make_waypoints_rectangular( dp_a=[0.25, 0.0, 0.1], dp_b=[0.45, -0.2, 0.1], rpy=[0.0,0.0,0.0] ):
    '''
    Create a array of rectangular position coordinates
    from diagonal points of A and B.
    
    @type  dp_a : [float, float, float]
    @param dp_a : Position (x,y,z) [m] coordinates of a diagonal point A
    @type  dp_a : [float, float, float]
    @param dp_a : Position (x,y,z) [m] coordinates of a diagonal point B
    @type  rpy  : [float, float, float]
    @param rpy  : Roll/Pitch/Yaw angle of the poses
    
    @rtype  : [ Pose, Pose, ... ]
    @return : Array of Pose waypoints
              Length of 5 (including an end position same as the start position)
    '''
    
    wpts = []
    pose_target = Pose()
    
    quat = tf.transformations.quaternion_from_euler( rpy[0], rpy[1], rpy[2] )
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    
    pose_target.position.x = dp_a[0]
    pose_target.position.y = dp_a[1]
    pose_target.position.z = dp_a[2]
    wpts.append( copy.deepcopy( pose_target ) )
    
    pose_target.position.y = dp_b[1]
    wpts.append( copy.deepcopy( pose_target ) )
    
    pose_target.position.x = dp_b[0]
    pose_target.position.z = dp_b[2]
    wpts.append( copy.deepcopy( pose_target ) )
    
    pose_target.position.y = dp_a[1]
    wpts.append( copy.deepcopy( pose_target ) )
    
    pose_target.position.x = dp_a[0]
    pose_target.position.z = dp_a[2]
    wpts.append( copy.deepcopy( pose_target ) )
    
    return wpts


def make_waypoints_circular( center=[0.3, -0.2, 0.1], radius=0.1 ,steps=12, rpy=[0.0,0.0,0.0] ):
    '''
    Create a array of circular position coordinates
    from a center position and a radius divided by steps.
    
    @type  center : [float, float, float]
    @param center : Position (x,y,z) [m] coordinates of a circle center
    @type  radius : float
    @param radius : Radius length [m]
    @type  steps  : int
    @param steps  : Number of steps for dividing a circle
    @type  rpy    : [float, float, float]
    @param rpy    : Roll/Pitch/Yaw angle of the poses
    
    @rtype  : [ Pose, Pose, ... ]
    @return : Array of Pose waypoints
              Length of steps+1 (including an end position same as the start position)
    '''
    
    wpts = []
    pose_target = Pose()
    
    quat = tf.transformations.quaternion_from_euler( rpy[0], rpy[1], rpy[2] )
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    
    step_rad = 2 * math.pi / steps
    for i in range(steps+1):
        ang_rad = step_rad * i
        pose_target.position.x = center[0] - radius * math.cos(ang_rad)
        pose_target.position.y = center[1] + radius * math.sin(ang_rad)
        pose_target.position.z = center[2]
        wpts.append( copy.deepcopy( pose_target ) )
    
    return wpts


def get_ros_version():
    
    new_proc = subprocess.Popen(["rosversion", "-d"], stdout=subprocess.PIPE)
    ros_version = new_proc.communicate()[0]
    
    return ros_version.rstrip('\n')
    

def normalize_orientation( pose ):
    
    q_orig = [ pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w ]
    q_norm = q_orig / numpy.linalg.norm( q_orig )
    pose.orientation.x = q_norm[0]
    pose.orientation.y = q_norm[1]
    pose.orientation.z = q_norm[2]
    pose.orientation.w = q_norm[3]
    
    rospy.loginfo( "Orientation Normalized" )
    
    return pose
    
