#!/usr/bin/env python
import roslib; roslib.load_manifest('mybot_navigation')
import rospy
import actionlib
from robot_project.msg import ObjPoseArray, ObjPose
import time
from move_base_msgs.msg import * #move_base_msgs
import numpy as np
from std_msgs.msg import String
import subprocess
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi as PI
from math import atan2
import tf

def check_voice():
    if rospy.has_param('find_obj'):
        return rospy.get_param('find_obj')
    else:
        return 'e'


# Simpole_Move function is used to conduct 'simple' mode of motion
def Simple_Move(x_in, y_in, th_in):
    dis = np.linalg.norm([Px-x_in, Py-y_in])
    if dis < 0.2:
        print 'Target too close to the current position, omit the motion'
    else:
        time.sleep(0.01)
        th = atan2(y_in, x_in)
        moveto(0.0, 0.0, th)
        dis = np.linalg.norm([x_in, y_in])
        moveto(float(dis), 0.0, 0.0)
        time.sleep(0.5)
        moveto(0.0, 0.0, float(th_in-th))


def Position(odom_data):
    global pose, Px, Py, Th
    pose = odom_data.pose.pose #  the x,y,z pose and quaternion orientation
    Px = pose.position.x
    Py = pose.position.y
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w]) 
    Th = -yaw


def moveto(x, y, th):
    rospy.sleep(0.2)
    #print P_pose.position.x
    current_x = Px
    current_y = Py
    current_Th = Th
    error_sum = 0
    kp = 0.2
    ki = 0.015
    error = 0
    while(abs(x - math.sqrt((Px - current_x)**2 + (Py - current_y)**2)) > 0.01):
        rospy.sleep(0.1)
        velocity = Twist()
        error = x - math.sqrt((Px - current_x)**2 + (Py - current_y)**2)
        
        velocity.linear.x = error * kp + error_sum * ki
        if velocity.linear.x > 0.25:
            velocity.linear.x = 0.25
        error_sum += error
        vel_pub.publish(velocity)
        print "linear error = ", error
    velocity = Twist()
    velocity.linear.x = 0
    vel_pub.publish(velocity)
    print "stop"
    
    
    current_Th = Th
    
    kp = 0.2
    ki = 0.015
    error =  current_Th + th - Th
    """
    if current_Th + th > PI:
        if (current_Th + th + 2*PI)*Th < 0:
            error = current_Th + th - Th 
        else:
            error = current_Th + th - Th - 2*PI
            
    if current_Th + th > PI:
        if (current_Th + th - 2*PI)*Th < 0:
            error = current_Th + th - Th 
        else:
            error = current_Th + th - Th + 2*PI
    """

    error_sum = 0
    while(abs(error) > 0.1):
        rospy.sleep(0.1)
        velocity = Twist()
        error = current_Th + th - Th
        if current_Th + th < -PI:
            if (current_Th + th + 2*PI)*Th < 0:
                error = current_Th + th - Th 
            else:
                error = current_Th + th - Th + 2*PI
                
        if current_Th + th > PI:
            if (current_Th + th - 2*PI)*Th < 0:
                error = current_Th + th - Th 
            else:
                error = current_Th + th - Th - 2*PI
        velocity.angular.z = -(error * kp + error_sum * ki)
        error_sum += error
        if (velocity.angular.z) > 0.4:
            velocity.angular.z = 0.4
        if (velocity.angular.z) < -0.4:
            velocity.angular.z = -0.4
        vel_pub.publish(velocity)
        p_error = error
        #print error
        print 'angular error = ', error
    velocity = Twist()
    velocity.angular.z = 0
    vel_pub.publish(velocity)
    print "stop"


def get_obj_callback(data):
    global obj_list
    # print 'data'
    # print data
    obj_list = data.ObjPoseArray
    # print obj_list
    

def temp(voice_value):
    # print obj_list
    if voice_value == 'e' or voice_value == 'nothing':
        return

    rospy.set_param('find_obj', 'nothing')
    print 'voice_value = ', voice_value

    if voice_value != 'person':
        # run openpose
        rospy.set_param('use_openpose', True)
        point_result = ''
        while True:
            if rospy.has_param('point'):
                
                point_result = rospy.get_param('point')
                rospy.delete_param('point')
                rospy.set_param('use_openpose', False)
                print point_result
                if point_result == 'left':
                    print 'turn right'
                    time.sleep(4)
                    moveto(0, 0, 0.4*PI)
                elif point_result == 'right':
                    print 'turn left'
                    time.sleep(4)
                    moveto(0, 0, -0.4*PI)
                break

    # print 'obj_list = ', obj_list
    print type(obj_list)
    for obj in obj_list:
        # print obj
        if obj.Class == voice_value:
            print 'Found Object: ', obj.Class, 'pose = ( ', obj.x, ' , ', obj.y, ' )'
            rospy.set_param('is_moving', True)
            print 'move to ', obj.x, obj.y, 0.0
            moveto(obj.y-0.65, obj.x, 0.0)
            rospy.set_param('is_moving', False)
            time.sleep(0.5)
            break


if __name__ == '__main__':
    # declare global varibales
    frame_ID = 'map'
    obj_list = ''
    Px = 0        #---------------- robot x position
    Py = 0        #---------------- robot y position
    Th = 0        #---------------- robot theta position

    rospy.set_param('is_moving', False)

    rospy.init_node('main_controller', anonymous=True)
    rospy.loginfo('main_controller initialization')
    vel_pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber( '/robot_project/get_obj/obj_pose', ObjPoseArray, get_obj_callback)
    rospy.Subscriber('RosAria/pose',Odometry, Position)

    rate = rospy.Rate(10.0)
    # Simple_Move(0.4, 0.3, 0.0)
    while not rospy.is_shutdown():
        # print obj_list
        voice_value = check_voice()
        temp(voice_value)

        rospy.sleep(0)