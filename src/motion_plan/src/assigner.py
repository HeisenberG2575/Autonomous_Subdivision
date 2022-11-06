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
            found, pos, orient = my_client.recovery()
            i = 0
        if len(my_client.completed_list) == 6:
            rospy.loginfo("End Goal found")
            # success = my_client.move_to_goal(12,6,q=(0,0,1/np.sqrt(2),1/np.sqrt(2)))
            # if success:
            #     rospy.loginfo("Completed all goals")
            # else:
            #     rospy.loginfo("Failed")
            break
        found, pos, orient = my_client.arrow_detect(far=True)
        if found:
            # TODO reduce conversions
            orient = orient + 90 if orient < 0 else orient - 90
            q=(0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
            posx,posy,q = my_client.bot_to_map(pos[0], pos[1], q, frame="camera_link") #map frame
            if my_client.is_complete(posx, posy, q):
                rospy.loginfo("Already visited recently found Goal: " + str([posx, posy]))
                found = False
            else:
                my_client.add_arrow(posx, posy, q, color=(1,0,0))#Add Rviz arrow marker, map frame
                i = 1
                #rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))
                x,y,q_p = my_client.bot_to_map(0,0, (0,0,0,1))#bot location
                success = my_client.send_goal(*my_client.find_off_goal(\
                      posx,posy, q = q_p, offset = (-1.75,0,0,0)), frame = "map")
                rospy.sleep(1)
                dist = norm([x-posx, y-posy])
                while success == False and dist > 1.75:#keep checking if we are moving correctly
                    dist = norm([x-posx, y-posy])
                    success = my_client.send_goal(*my_client.find_off_goal(\
                        posx,posy, q = q_p, offset = (-1.75,0,0,0)), frame = "map")
                    rospy.sleep(1)
                    found, pos, orient = my_client.arrow_detect(far = dist > 2)
                    if found == False or posx is None:
                        found, pos, orient = my_client.recovery()
                    if found == False:
                        break
                    orient = orient + 90 if orient < 0 else orient - 90
                    q_p=(0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                    x,y,q_p = my_client.bot_to_map(0,0, q_p)#bot location, with perpendicular to arrow goal
                    dist = norm([x-posx, y-posy])
                    if my_client.is_complete(posx, posy, q_p):
                        rospy.loginfo("Already visited recently found Goal: " + str([posx, posy]))
                        found = False
                        break
                if found == True:
                    # my_client.cancel_goal()
                    # TODO change to 20
                    rospy.sleep(1)
                    success = my_client.move_to_goal(*my_client.find_off_goal(\
                        posx,posy, q = q_p, offset = (-1.6,0,0,0)), frame = "map")

                    found, pos, orient = my_client.arrow_detect(far=False)
                    if found == False or pos is None:
                        found, pos, orient = my_client.recovery()
                    if found == False:
                        continue
                    q=(0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                    posx,posy, q = my_client.bot_to_map(0, 0, q)
                    success = my_client.move_to_off_goal(posx,posy, q = q, frame = "map", off_dist = 1.3)
                    posx,posy, q = my_client.bot_to_map(pos[0], pos[1], q, frame="camera_link")
                    my_client.add_arrow(posx, posy, q, color=(0,1,0), pos_z = pos[2])#Add Rviz arrow marker, map frame
                    if success == True:
                        #my_client.add_arrow(*my_client.bot_to_map(posx, posy, q, frame="camera_link"), color=(0,1,1))
                        prev_x, prev_y, prev_q = posx, posy, q#map frame
                        #my_client.add_arrow(prev_x, prev_y, prev_q, (1,0,1))
                        my_client.add_to_completed(posx, posy, q)
                    else:
                        rospy.loginfo("Failed goal: " + str((posx, posy, q)))
        if not found:
            nearby_goal = just_ahead(prev_x,prev_y, prev_q, off_dist= 0.5 + 0.65*i)
            my_client.send_goal(*nearby_goal,frame="map")
            rospy.sleep(1.0)#Sleep for 1-2s and let the bot move towards the goal
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
