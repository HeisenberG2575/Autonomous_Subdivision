#!/usr/bin/env python
# Import the necessary libraries
import rospy  # Python library for ROS
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import Header
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import os, re

FRAME='camera_link_optical'

class ImagePublisher:
    def __init__(self, video_capture_path=None):
        rospy.init_node("ip_image_pub", anonymous=True)
        if video_capture_path:
            self.vid = cv2.VideoCapture(video_capture_path)
        else:
            self.get_camera()
            self.vid = cv2.VideoCapture(self.device_num)
        self.vid.set(cv2.CAP_PROP_FRAME_WIDTH,160)
        self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT,120)
        print(f'id: {self.ros_topic} w,h: {self.vid.get(cv2.CAP_PROP_FRAME_WIDTH)}, {self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT)}')
        self.counter=0
        self.br = CvBridge()
        rate = rospy.Rate(10)
        self.pub = rospy.Publisher(f'{self.ros_topic}', Image, queue_size=10)
        while not rospy.is_shutdown():
            ret, frame = self.vid.read()
            if not ret:
                print(f"id: {self.ros_topic}: ret == False")
            # rospy.loginfo(ret)
            # print(frame.shape)
            # frame=cv2.resize(frame,(256,144))
            # cv2.imshow("frame", frame)
            # cv2.waitKey(10)
            ros_img = self.br.cv2_to_imgmsg(frame,header=Header(seq=self.counter,stamp=rospy.Time.now(),frame_id=FRAME))
            self.pub.publish(ros_img)
            rate.sleep()
            self.counter+=1
        self.vid.release()
        # cv2.destroyAllWindows()

    def get_camera(self):
        self.device_num = rospy.get_param("~camera_number", 0)
        self.ros_topic = rospy.get_param("~ros_topic", 1)
        DEFAULT_CAMERA_NAME = f"/dev/video{self.device_num}"
        # print(DEFAULT_CAMERA_NAME)
        if os.path.exists(DEFAULT_CAMERA_NAME):
            device_path = os.path.realpath(DEFAULT_CAMERA_NAME)
            device_re = re.compile("\/dev\/video(\d+)")
            info = device_re.match(device_path)
            if info:
                self.device_num = int(info.group(1))
                print("Using default video capture device on /dev/video"\
                      + str(self.device_num))




if __name__ == "__main__":
    x = ImagePublisher()
