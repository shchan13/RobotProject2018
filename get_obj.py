#!/usr/bin/env python

##############################################################
# Please do AMCL localization first before running this code.
##############################################################

import rospy
import rosnode
from darknet_ros_msgs.msg import *
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from robot_project.msg import ObjPoseArray, ObjPose
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import tf

class VideoFrames:
    """
    Reference : ros-video-recorder
    https://github.com/ildoonet/ros-video-recorder/blob/master/scripts/recorder.py
    """
    def __init__(self, image_topic):
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback_image, queue_size=1)
        self.bridge = CvBridge()
        self.frames = []

    def callback_image(self, data):
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(data, 'mono16')
            cv_image = self.bridge.imgmsg_to_cv2(data)
            
        except CvBridgeError as e:
            rospy.logerr('Converting Image Error. ' + str(e))
            return

        self.frames.append((data.header.stamp, cv_image))

    def get_latest(self, at_time, remove_older=True):
        fs = [x for x in self.frames if x[0] <= at_time]
        if len(fs) == 0:
            return None

        f = fs[-1]
        if remove_older:
            self.frames = self.frames[len(fs) - 1:]

        return f[1]


def camera2pose(depth, pixel_x):
    obj_x = depth*np.tan(fov/2)*(pixel_x - cam_width) / cam_width
    obj_x /= 1000
    obj_y = depth / 1000

    """ 
    # Convert robot frame to map frame
    print '=============== Get amcl_pose ==============='
    print 'robot_pose_x = ', robot_pose_x
    print 'robot_pose_y = ', robot_pose_y
    print 'robot_angle = ', robot_angle
    print '============================================='

    if robot_pose_x != -1000.0 and robot_pose_y != -1000.0 and robot_angle != -1000.0:
        print 'Convert to map frame'
        
        T_robot2map = np.array([[np.cos(robot_angle), -np.sin(robot_angle), robot_pose_x], 
                                [np.sin(robot_angle),  np.cos(robot_angle), robot_pose_y],
                                [0.0                ,  0.0                , 1.0]])
        
        print 'obj_x, obj_y = ', obj_x, obj_y
        obj_x = np.cos(robot_angle)*obj_x - np.sin(robot_angle)*obj_y + robot_pose_x
        obj_y = np.sin(robot_angle)*obj_x + np.cos(robot_angle)*obj_y + robot_pose_y 
        print 'After transform ...'
        print 'obj_x, obj_y = ', obj_x, obj_y
        time.sleep(5)
    """
    return obj_x, obj_y


def draw_map(pose_x, pose_y, class_name, temp_map):
    # global: temp_map
    # pose_x, pose_y: unit = (m)
    temp_x = int(pose_x*100)*2 + 256 # m -> 1 cm = 5 pixel
    temp_y = int(pose_y*100)*2
    # print 'temp_x, temp_y = ', temp_x, temp_y
    cv2.circle(temp_map, (temp_x, temp_y), 3, (255,0,0), 2)
    cv2.putText(temp_map, class_name, (temp_x+5, temp_y+5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
    return


def bd_callback(data):
    # get image with pose time
    t = data.header.stamp
    depth_img = df.get_latest(t, remove_older=True)
    if depth_img is None:
        rospy.logwarn('No received depth images.')
        return

    # print 'depth_img shape = ',  depth_img.shape
    obj_list = []
    temp_map = np.ones((512,512,3), np.uint8)*255
    cv2.circle(temp_map, (256, 0), 3, (0,0,255), 2)

    for obj in data.bounding_boxes:
        # class_list.append(obj.Class)
        # print obj.ymin, ' ', obj.ymax, ' ', obj.xmin, ' ',obj.xmax
        bound_depth_img = np.copy(depth_img[obj.ymin:obj.ymax, obj.xmin:obj.xmax])
        # print 'bound_depth_img shape = ', bound_depth_img.shape
        bound_depth_img[bound_depth_img > 5000] = 5000 # max range = 5m
        # temp_pub.publish(cv_bridge.cv2_to_imgmsg(bound_depth_img))

        slide_width = np.min([bound_depth_img.shape[0], bound_depth_img.shape[1]]) // 4
        # print 'bound_depth_img shape = ', bound_depth_img.shape
        # print 'slide_width = ', slide_width 

        sxmin = bound_depth_img.shape[1]//2 - slide_width//2
        sxmax = bound_depth_img.shape[1]//2 + slide_width//2
        symin = bound_depth_img.shape[0]//2 - slide_width//2
        symax = bound_depth_img.shape[0]//2 + slide_width//2
        
        search_box = bound_depth_img[symin:symax, sxmin:sxmax]
        depth_val = np.mean(search_box)
        
        obj_pixel_x = obj.xmin+bound_depth_img.shape[1]//2
        depth_img = cv2.circle(depth_img, (obj.xmin+bound_depth_img.shape[1]//2, obj.ymin+bound_depth_img.shape[0]//2), 3, 100, 2)
        depth_img = cv2.rectangle(depth_img, (obj.xmin+sxmin, obj.ymin+symin), (obj.xmin+sxmax, obj.ymin+symax), 200, 2)
        depth_img = cv2.rectangle(depth_img, (obj.xmin, obj.ymin), (obj.xmax, obj.ymax), 255, 2)

        # Convert camera data to pose
        obj_x, obj_y = camera2pose(depth_val, obj_pixel_x)
        # print obj.Class, ' ', 'depth = ', depth_val
        # print 'position = ( ', obj_x, ' , ', obj_y, ' )'

        temp_obj = ObjPose()
        temp_obj.Class = obj.Class
        temp_obj.x = obj_x
        temp_obj.y = obj_y

        draw_map(temp_obj.x, temp_obj.y, temp_obj.Class, temp_map)

        obj_list.append(temp_obj)

    # Publish obj_list
    # print obj_list
    obj_pose_array = obj_list
    obj_pose_pub.publish(obj_pose_array)

    
    temp_pub.publish(cv_bridge.cv2_to_imgmsg(depth_img))
    temp_map_pub.publish(cv_bridge.cv2_to_imgmsg(temp_map, 'bgr8'))         
        
    return


def handle_robot_pose(data):
    robot_pose_x = data.pose.pose.position.x
    robot_pose_y = data.pose.pose.position.y
    euler_angle = tf.transformations.euler_from_quaternion(data.pose.pose.orientation)
    robot_angle = euler_angle[2]

    return


if __name__ == '__main__':
    rospy.init_node('get_obj', anonymous=True)
    rospy.loginfo('get_obj initialization')    

    # initialization (global variables)
    depth_topic = '/camera/depth_registered/image_raw'
    fov = 57*np.pi / 180
    cam_width = 320
    
    # robot_pose_x = -1000.0
    # robot_pose_y = -1000.0
    # robot_angle  = -1000.0

    obj_pose_array = ObjPoseArray()
    cv_bridge = CvBridge()
    df = VideoFrames(depth_topic) # get depth image
    
    print 'Wait for depth_topic'
    rospy.wait_for_message(depth_topic, Image, timeout=30)
    # print 'Waiting for amcl'
    # rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=60)
    print 'Start!'

    # Publisher
    temp_pub = rospy.Publisher('/robot_project/get_obj/temp', Image, queue_size=1)
    temp_map_pub = rospy.Publisher('/robot_project/get_obj/temp_map', Image, queue_size=1)
    obj_pose_pub = rospy.Publisher('/robot_project/get_obj/obj_pose', ObjPoseArray, queue_size=100)

    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, bd_callback, queue_size=100)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, handle_robot_pose, queue_size=1)
    
    rospy.spin()