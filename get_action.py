#!/usr/bin/env python
import time
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from tfpose_ros.msg import Persons, Person, BodyPartElm
from tf_pose.estimator import Human, BodyPart, TfPoseEstimator

import numpy as np 
import actionlib
import roslib; roslib.load_manifest('mybot_navigation')
from tf.transformations import quaternion_from_euler
# from geometry_msgs.msg import Twist, PosewithCovarianceStamp, Pose2D
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
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

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



def joint_angle(body_part1, body_part2, body_part3):
    # calculate the angle of part2

    def get_angle(vec1, vec2):
        # vec1: np.array([x,y])
        # return: theta in rad
        cos_theta = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
        return np.arccos(cos_theta) * 180/np.pi 

    vec21 = np.array([body_part2.x - body_part1.x, body_part2.y - body_part1.y])
    vec23 = np.array([body_part2.x - body_part3.x, body_part2.y - body_part3.y])
    return get_angle(vec21, vec23)


def check_arm_raise(sholder, arm_up):
    # input: sholder joints(5 or 2) / arm up joint(6 or 3)
    if arm_up.y < sholder.y and (arm_up.x - sholder.x) < 0.1:
        return True
    else:
        return False


def get_action(data):
    # get image with pose time
    t = data.header.stamp
    image = vf.get_latest(t, remove_older=True)
    if image is None:
        rospy.logwarn('No received images.')
        return

    h, w = image.shape[:2]
    if resize_ratio > 0:
        image = cv2.resize(image, (int(resize_ratio*w), int(resize_ratio*h)), interpolation=cv2.INTER_LINEAR)

    # ros topic to Person instance
    humans = []

    for p_idx, person in enumerate(data.persons):
        human = Human([])
        for body_part in person.body_part:
            part = BodyPart('', body_part.part_id, body_part.x, body_part.y, body_part.confidence)
            human.body_parts[body_part.part_id] = part

        humans.append(human)

    if len(humans) > 0:
        master = humans[0]
        # print 'master = ', master
        l_up_angle = l_low_angle = r_up_angle = r_low_angle = 0.0
        
        # check point left
        if master.body_parts.get(1) != None and master.body_parts.get(5) != None and master.body_parts.get(6) != None and master.body_parts.get(7) != None:
            l_up_angle = joint_angle(master.body_parts[1], master.body_parts[5], master.body_parts[6]) 
            l_low_angle = joint_angle(master.body_parts[5], master.body_parts[6], master.body_parts[7])

            if l_up_angle > 140. and l_low_angle > 160. and not check_arm_raise(master.body_parts[5], master.body_parts[6]):
                print 'point left' # relative to human
                rospy.set_param('point', 'left')


        # check point right
        if master.body_parts.get(1) != None and master.body_parts.get(2) != None and master.body_parts.get(3) != None and master.body_parts.get(4) != None:
            r_up_angle = joint_angle(master.body_parts[1], master.body_parts[2], master.body_parts[3]) 
            r_low_angle = joint_angle(master.body_parts[2], master.body_parts[3], master.body_parts[4])

            if r_up_angle > 140. and r_low_angle > 160. and not check_arm_raise(master.body_parts[2], master.body_parts[3]):
                print 'point right' # relative to human
                rospy.set_param('point', 'right')
        

    # draw
    image = TfPoseEstimator.draw_humans(image, humans, imgcopy=False)
    pub_img.publish(cv_bridge.cv2_to_imgmsg(image, 'bgr8'))


if __name__ == '__main__':
    rospy.loginfo('initialization+')
    rospy.init_node('TfAction', anonymous=True)

    # topics params
    image_topic = rospy.get_param('~camera', '/camera/rgb/image_color')
    pose_topic = rospy.get_param('~pose', '/pose_estimator/pose')

    resize_ratio = float(rospy.get_param('~resize_ratio', '-1'))

    # check whether action_flag exists
    if not rospy.has_param('action_flag'):
        print 'no voice trigger'
        rospy.set_param('action_flag', True)

    # publishers
    pub_img = rospy.Publisher('/robot_project/TfAction/output', Image, queue_size=1)

    # initialization
    cv_bridge = CvBridge()
    vf = VideoFrames(image_topic)
    rospy.wait_for_message(image_topic, Image, timeout=30)
    print 'get image_topic'

    # subscribers
    rospy.Subscriber(pose_topic, Persons, get_action, queue_size=1)

    # run
    rospy.spin()
