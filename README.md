# RobotProject2018
Robot project for GPGPU2018

## 1. Webpage for detailed description:
   https://a9451406.wixsite.com/gpgpurobotproject

## 2. Installation:

   (1) Please install ROS, tf-openpose, YOLOv3 first.
   
   (2) Create a workspace for ROS named as catkin_ws/ and also create a package named robot_project.
   
   (3) Grab main_controller.py, get_obj.py, voice_reg.py to robot_project/script; ObjPose.msg and ObjPoseArray.msg to robot_project/msg
   
   (4) Replace robot_project/CMakeList.txt and robot_project/package.xml with ours
   
   (5) Grab get_action.py, broadcaster.py under tf-openpose folder
  
3. Execution
   $ roscore
   
   $ roslaunch darknet_ros darknet_ros.launch
   
   $ python catkin_ws/src/robot_project/script/main_controller.py
   
   $ python catkin_ws/src/robot_project/script/get_obj.py
   
   $ python catkin_ws/src/robot_project/script/voice_reg.py
   
   $ python YOUR_PATH/tf-openpose/get_action.py
   
   $ python YOUR_PATH/tf-openpose/broadcatser_ros.py
  
4. Reference:
   (1) ROS: http://wiki.ros.org/kinetic/Installation
   (2) YOLOv3: https://github.com/leggedrobotics/darknet_ros
   (3) tf-openpose: https://github.com/ildoonet/tf-pose-estimation
   (4) ROS voice message (Android): https://play.google.com/store/apps/details?id=org.ros.android.android_voice_message&hl=zh_TW
