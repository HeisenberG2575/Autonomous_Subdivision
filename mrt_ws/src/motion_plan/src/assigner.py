#!/usr/bin/env python

from functions import *
import rospy
from cv2 import destroyAllWindows
import sys
 


def main_node():
    
    my_client = client()
    rate = rospy.Rate(5)
    prev_x,prev_y, prev_q = my_client.bot_to_map(0,0, (0,0,0,1)) #prev always in map frame
    # = (0,0,0,1)#TODO change prev here to initial bot location which may not be 0
    #my_client.move_to_goal(0.5,0)
    #x,y,q =  1,0,(0,0,np.sin(np.pi/8), np.cos(np.pi/8))
    #print(x,y,q)
    #my_client.move_to_off_goal(*my_client.bot_to_map(x,y,q))
    #print(x,y,q)
    i=1 #iteration for moving just ahead of the previous goal

    while not rospy.is_shutdown():
        if len(my_client.completed_list) == 5:
            rospy.loginfo("End Goal found")
            success = my_client.move_to_goal(12,6,q=(0,0,1/np.sqrt(2),1/np.sqrt(2)))
            if success:
                rospy.loginfo("Completed all goals")
            else:
                rospy.loginfo("Failed")
            break
        found, theta, orient = my_client.arrow_detect()#theta and orient wrt forward direction, in degree
        if found:
            posx,posy = my_client.find_obs_lidar(theta)
            if posx is None:
                rospy.loginfo("Arrow detected but not found in LIDAR. Check width/error/arrow detection")
                found = False
            
            else:
                q=(0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                #TODO Add a check if arrow found is nearly in the direction of the previous arrow(or add a warning if it is)
                if my_client.is_complete(posx,posy,q):
                    rospy.loginfo("Already visited recently found Goal: " + str([posx,posy]))
                else:
                    i = 1
                    #rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))
                    posx, posy, q = my_client.bot_to_map(posx, posy, q)#map frame
                    my_client.add_arrow(posx, posy, q, color=(0,1,0))#Add Rviz arrow marker, map frame
                    success = my_client.move_to_off_goal(posx,posy, q = q, frame = "map", off_dist = 1)
                    if success == True:
                        #my_client.add_arrow(*my_client.bot_to_map(posx, posy, q), color=(0,1,1))
                        prev_x, prev_y, prev_q = posx, posy, q#map frame
                        #my_client.add_arrow(prev_x, prev_y, prev_q, (1,0,1))
                        my_client.add_to_completed(posx, posy, q)
                    else:
                        rospy.loginfo("Failed goal: " + str((posx, posy, q)))
        if not found:
            nearby_goal = just_ahead(prev_x,prev_y, prev_q, off_dist= 0.5 + 0.6*i)
            my_client.send_goal(*nearby_goal,frame="map")
            rospy.sleep(1)#Sleep for 1-2s and let the bot move towards the goal
            i+=1
    	rate.sleep()

    # Close down the video stream when done
    destroyAllWindows()

if __name__ == '__main__':
	try:
		main_node()
	except rospy.ROSInterruptException:
		print('Exiting... ')
	except:
		print("Unexpected error:", sys.exc_info()[0])
		raise
