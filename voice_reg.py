#!/usr/bin/env python
import rospy
from jsk_gui_msgs.msg import VoiceMessage
import numpy as np 
from std_msgs.msg import String

def voice_callback(data):
    sep = data.texts[0].split(' ')
    # print yolo_obj
    # print 'sep = ', sep

    if 'come' and 'to' and ('me' or 'Me') in sep:
        # voice_pub.publish('me')
        rospy.set_param('find_obj', 'person')
    elif 'grab' in sep:
        check_yolo = False
        for i in sep:
            # print i
            if i in yolo_obj:
                rospy.set_param('find_obj', i) # grab a cup
                check_yolo = True
                break
        if check_yolo == False:
            rospy.set_param('find_obj', 'nothing')

        
    else:
        rospy.set_param('find_obj', 'nothing')

    return

if __name__ == '__main__':
    yolo_obj = ['person','bicycle','car','motorbike','aeroplane','bus','train','truck','boat','traffic light','fire hydrant','stop sign','parking meter','bench','bird','cat','dog','horse','sheep','cow','elephant','bear','zebra','giraffe','backpack','umbrella','handbag','tie','suitcase','frisbee','skis','snowboard','sports ball','kite','baseball bat','baseball glove','skateboard','surfboard','tennis racket','bottle','wine glass','cup','fork','knife','spoon','bowl','banana','apple','sandwich','orange','broccoli','carrot','hot dog','pizza','donut','cake','chair','sofa','pottedplant','bed','diningtable','toilet','tvmonitor','laptop','mouse','remote','keyboard','cell phone','microwave','oven','toaster','sink','refrigerator','book','clock','vase','scissors','teddy bear','hair drier','toothbrush']
    rospy.init_node('voice_reg', anonymous=True)
    rospy.loginfo('voice_reg initialization')
    # voice_pub = rospy.Publisher('/robot_project/voice_command', String, queue_size=10)
    rospy.Subscriber('/Tablet/voice', VoiceMessage, voice_callback)
    rospy.spin()