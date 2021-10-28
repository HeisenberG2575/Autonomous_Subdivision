#! /usr/bin/env python

import rospy
import numpy as np
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion
from tf import transformations
from nav_msgs.msg import OccupancyGrid


class client():
      def __init__(self):
            
            rospy.init_node('map_update')
            rospy.loginfo('assigner init')
            #define a client for to send goal requests to the move_base server through a SimpleActionClient
            self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            #wait for the action server to come up
            while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                  rospy.loginfo("Waiting for the move_base action server to come up")
            self.mapData = OccupancyGrid()
            rospy.Subscriber('/map', OccupancyGrid, self.mapCallBack)
            rospy.Rate(5).sleep()#rospy.spin()

      def mapCallBack(self,data):
            self.mapData=data

      def move_to_goal(self, xGoal,yGoal, quaternion=None, frame="link_chassis"):
            #relative to the bot location
            #quaternion is a 4-tuple/list-x,y,z,w or Quaternion

            goal = MoveBaseGoal()

            #set up the frame parameters
            goal.target_pose.header.frame_id = frame
            goal.target_pose.header.stamp = rospy.Time.now()

            # moving towards the goal*/

            goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
            if quaternion is None:
                  goal.target_pose.pose.orientation = Quaternion(0,0,0,1)
            elif isinstance(quaternion, list) or isinstance(quaternion, tuple):
                  goal.target_pose.pose.orientation = Quaternion(*quaternion)
            elif isinstance(quaternion,Quaternion):
                  goal.target_pose.pose.orientation = quaternion
            else:
                  print('Quaternion in incorrect format')
                  goal.target_pose.pose.orientation = Quaternion(0,0,0,1)#default behavior

            rospy.loginfo("Sending goal location ...")
            self.ac.send_goal(goal)

            self.ac.wait_for_result(rospy.Duration(60))

            if(self.ac.get_state() ==  GoalStatus.SUCCEEDED):
                  rospy.loginfo("You have reached the destination")
                  return True

            else:
                  rospy.loginfo("The robot failed to reach the destination")
                  return False

      def move_to_off_goal(self,xGoal,yGoal, off_dist=1, quaternion=None, frame="link_chassis"):
            if quaternion is None:
                  q = Quaternion(0,0,0,1)
            elif isinstance(quaternion, list) or isinstance(quaternion, tuple):
                  q = Quaternion(*quaternion)
            elif isinstance(quaternion,Quaternion):
                  q = quaternion
            else:
                  print('Quaternion in incorrect format')
                  q = Quaternion(0,0,0,1)
            #qp = transformations.quaternion_multiply(q, [1/np.sqrt(2),0,0,1/np.sqrt(2)] )
            #rotate by 90 deg with z as the axis, to get q perpendicular
            offset = transformations.quaternion_multiply((q.x,q.y,q.z,q.w), (0,off_dist,0,0))#move by 0.5, qpq^-1, change relative goal when stuck, here
            offset = transformations.quaternion_multiply(offset, transformations.quaternion_inverse((q.x,q.y,q.z,q.w)))
            x1, y1 = xGoal+offset[0],yGoal+offset[1]
            cell1=get_cell_status(self.mapData, [x1, y1])
            x2, y2 = xGoal-offset[0],yGoal-offset[1]
            cell2=get_cell_status(self.mapData, [x2, y2])
            if cell1==0:
                  if cell2==0:
                        x,y  = [x1,y1] if (x1**2+y1**2 < x2**2+y2**2) else [x2,y2]
                  else:
                        x,y = x1,y1
            else:
                  if cell2==0:
                        x,y = x2,y2
                  else:
                        x,y  = [x1, y1] if (x1**2+y1**2 < x2**2+y2**2) else [x2,y2]
            self.move_to_goal(x,y,q, frame)

      def cancel_goal(self):
            self.ac.cancel_goal()
            rospy.loginfo('goal cancelled')


def get_cell_status(mapData, pt):
      # returns grid value at point "pt"- shape:(2)
      # map data:  100 occupied      -1 unknown       0 free
      resolution = mapData.info.resolution
      Xstartx = mapData.info.origin.position.x
      Xstarty = mapData.info.origin.position.y
      width = mapData.info.width
      Data = mapData.data
      print(resolution)

      index = (np.floor((pt[1]-Xstarty)/resolution)*width) + (np.floor((pt[0]-Xstartx)/resolution))

      if int(index) < len(Data):
            return Data[int(index)]
      else:
            return 100


def main():
      my_client = client()
      my_client.move_to_off_goal(1,0)


if __name__ == '__main__':
      try:
            main()
      except rospy.ROSInterruptException:
            print("Closing")
