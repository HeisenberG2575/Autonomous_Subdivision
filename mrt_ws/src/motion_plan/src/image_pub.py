#!/usr/bin/env python
import rospy
import numpy as np
import sys
import cv2
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def start_node():
    print("Starting arrow detection script")
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')
    pub = rospy.Publisher('/mrt/camera1/image_raw',Image, queue_size=10)
    rospy.Rate(1.0).sleep() 
    bridge = CvBridge()
    capture = cv2.VideoCapture(0)
    ret_val, frame = capture.read()
    if ret_val == False:
        print("image/video error")
    while ret_val:
        ret_val, frame = capture.read()
        if frame is not None:
            frame = np.uint8(frame)
        #frame = np.array(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        imgMsg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(imgMsg)
        key = cv2.waitKey(50)
        cv2.destroyAllWindows()
    
    
    #cv2.imshow("image", img)
    #cv2.waitKey(2000)
    
    
    # while not rospy.is_shutdown():
    #      # 1 Hz

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass


