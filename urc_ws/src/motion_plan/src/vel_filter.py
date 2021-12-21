#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class filter():
  def __init__(self):
      rospy.init_node('vel_filter', anonymous=True)
      self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
      self.pub = rospy.Publisher('/cmd_vel_filtered',Twist,queue_size=10)
      rospy.spin()
      
  def callback(self, data):
      new_vel = Twist()
      if abs(data.angular.z) > 0.2 or abs(data.linear.x)<abs(data.angular.z):
        new_vel.angular.z = data.angular.z
      else :
        new_vel.linear.x = data.linear.x
      self.pub.publish(new_vel)
      
if __name__ == '__main__':
      try:
            my_filter = filter()
      except rospy.ROSInterruptException:
            print("Closing vel_filter")
