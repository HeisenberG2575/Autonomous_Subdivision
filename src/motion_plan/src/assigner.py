#!/usr/bin/env python

from functions import *
import rospy
from cv2 import destroyAllWindows
import sys



def main_node():
    
    my_client = client()
    rate = rospy.Rate(5)
    prev_x,prev_y, prev_q = my_client.bot_to_map(0,0, (0,0,0,1)) #prev always in map frame
    i=1 #iteration for moving just ahead of the previous goal

    while not rospy.is_shutdown():
        if i > 8:
            found, theta, orient, posx, posy = my_client.recovery()
            i = 0
        if len(my_client.completed_list) == 6:
            rospy.loginfo("End Goal found")
            # success = my_client.move_to_goal(12,6,q=(0,0,1/np.sqrt(2),1/np.sqrt(2)))
            # if success:
            #     rospy.loginfo("Completed all goals")
            # else:
            #     rospy.loginfo("Failed")
            break
        found, theta, orient = my_client.arrow_detect(far=True)
        if found:
            posx,posy = my_client.find_obs_lidar(theta)#map frame
            if posx is None:
                rospy.loginfo("Arrow detected but not found in LIDAR. Check width/error/arrow detection")
                found = False
            
            else:
                if my_client.is_complete(posx, posy):
                    rospy.loginfo("Already visited recently found Goal: " + str([posx, posy]))
                    found = False
                else:
                    i = 1
                    #rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))
                    x,y,q_p = my_client.bot_to_map(0,0, (0,0,0,1))#bot location
                    success = my_client.send_goal(*my_client.find_off_goal(posx,posy, q = q_p, offset = (-1.5,0,0,0)), frame = "map")
                    rospy.sleep(1)
                    dist = norm([x-posx, y-posy])
                    while success == False and dist > 1.75:#keep checking if we are moving correctly
                        dist = norm([x-posx, y-posy])
                        success = my_client.send_goal(*my_client.find_off_goal(posx,posy, q = q_p, offset = (-1.5,0,0,0)), frame = "map")
                        rospy.sleep(1)
                        found, theta, orient = my_client.arrow_detect(far = dist > 2)
                        posx,posy = my_client.find_obs_lidar(theta)
                        if found == False or posx is None:
                            found, theta, orient, posx, posy = my_client.recovery()
                        if found == False:
                            break
                        orient = orient + 90 if orient < 0 else orient - 90
                        q_p=(0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                        x,y,q_p = my_client.bot_to_map(0,0, q_p)#bot location, with perpendicular to arrow goal
                        dist = norm([x-posx, y-posy])
                        if my_client.is_complete(posx, posy):
                            rospy.loginfo("Already visited recently found Goal: " + str([posx, posy]))
                            # found, theta, orient, posx, posy = my_client.recovery()
                            found = False
                            break
                    if found == True:
                        # my_client.cancel_goal()
                        rospy.sleep(5)
                        success = my_client.move_to_goal(*my_client.find_off_goal(posx,posy, q = q_p, offset = (-1.25,0,0,0)), frame = "map")

                        found, theta, orient = my_client.arrow_detect(far = False)
                        posx,posy = my_client.find_obs_lidar(theta)
                        if found == False or posx is None:
                            found, theta, orient, posx, posy = my_client.recovery()
                        if found == False:
                            continue
                        q=(0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                        _, _, q = my_client.bot_to_map(0, 0, q)
                        success = my_client.move_to_off_goal(posx,posy, q = q, frame = "map", off_dist = 1)
                        my_client.add_arrow(posx, posy, q, color=(0,1,0))#Add Rviz arrow marker, map frame
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
            rospy.sleep(1.5)#Sleep for 1-2s and let the bot move towards the goal
            i+=1
    	# rate.sleep()

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
