#!/usr/bin/env python
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import os, re
DEFAULT_CAMERA_NAME = '/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_EA2B8C60-video-index0'

device_num = 0
if os.path.exists(DEFAULT_CAMERA_NAME):
    device_path = os.path.realpath(DEFAULT_CAMERA_NAME)
    device_re = re.compile("\/dev\/video(\d+)")
    info = device_re.match(device_path)
    if info:
        device_num = int(info.group(1))
        print("Using default video capture device on /dev/video" + str(device_num))


class ImagePublisher():
  def __init__(self, video_capture_path=None):
    if video_capture_path:
      self.vid = cv2.VideoCapture(video_capture_path)
    else:
      self.vid = cv2.VideoCapture(device_num)
    rospy.init_node("ip_image_pub", anonymous=True)
    self.br = CvBridge()  
    rate = rospy.Rate(10)
    self.pub = rospy.Publisher('/mrt/camera1/image_raw', Image)
    while not rospy.is_shutdown():
        ret, frame = self.vid.read()
        # print(ret)
        ros_img = self.br.cv2_to_imgmsg(frame)
        self.pub.publish(ros_img)
        rate.sleep()
        
if __name__ == "__main__":
    x = ImagePublisher()
