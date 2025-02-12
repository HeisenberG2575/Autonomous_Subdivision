#!/usr/bin/env python

import sys, rospkg, os
import numpy as np
import importlib
from numpy.linalg import norm

path = rospkg.RosPack().get_path("motion_plan")
sys.path.append(path + "/src")
import functions

# from functions import *
import rospy
from cv2 import destroyAllWindows

my_client = functions.client()
eps=1
search_offset=2

def main_node():

    rate = rospy.Rate(5)
    prev_x, prev_y, prev_q = my_client.bot_to_map(
        0, 0, (0, 0, 0, 1)
    )  # prev always in map frame
    olat, olon = my_client.olat, my_client.olon
    print(olat,olon)
    # = (0,0,0,1)#TODO change prev here to initial bot location which may not be 0
    # my_client.move_to_goal(0.5,0)
    # x,y,q =  1,0,(0,0,np.sin(np.pi/8), np.cos(np.pi/8))
    # print(x,y,q)
    # my_client.move_to_off_goal(*my_client.bot_to_map(x,y,q))
    # print(x,y,q)
    i = 1  # iteration for moving just ahead of the previous goal
    # while True:
    #     print(my_client.ar_detect())
    #     rospy.sleep(1)
    # gps_goal_lat, gps_goal_lon = my_client.xy2gps(-3, -2)
    #gps_goal_lat, gps_goal_lon = my_client.xy2gps(2,0)
    #gps_goals(0,gps_goal_lat, gps_goal_lon)
    #gps_goal_lat, gps_goal_lon = my_client.xy2gps(4,0)
    #gps_goals(0,gps_goal_lat, gps_goal_lon)
    #my_client.flash_green()
    # gps_goal_lat, gps_goal_lon = my_client.xy2gps(0,0)
    #gps_goal_lat, gps_goal_lon = 19.13311, 72.916715
    #gps_goals(0,gps_goal_lat, gps_goal_lon)
    x,y,q = my_client.bot_to_map(5,0,None)
    # gps_goal_lat, gps_goal_lon = my_client.xy2gps(-5,0)
    # gps_goals(0, gps_goal_lat, gps_goal_lon)
    # gps_goal_lat, gps_goal_lon = my_client.xy2gps(0,0)
    #gps_goals(0, gps_goal_lat, gps_goal_lon)
    gps_goal_lat, gps_goal_lon = my_client.xy2gps(x,y)
    gps_goals(2, gps_goal_lat, gps_goal_lon)


def gps_goals(type, lat, lon):
    goal_xy_x,goal_xy_y=my_client.gps2xy(lat,lon)
    if type==0:
        success = my_client.move_to_goal(*my_client.gps2xy(lat,lon),q=(0,0,1,0))
        if success:
            rospy.loginfo("Reached Post")
        else:
            rospy.loginfo("Failed")

    elif type==1:
        counter=0
        while not rospy.is_shutdown():
            # AR Tag Detection
            if counter==0:
                my_client.move_to_goal(*my_client.gps2xy(lat,lon),q=(0,0,0,1))
                rospy.sleep(1)
            found, pts = my_client.ar_detect()  # theta and orient wrt forward direction, in degree
            # single post
            if found == 1:
                # posx, posy = my_client.find_obs_lidar(theta[0])
                posx, posy = pts[0][0],pts[0][1]
                # posx, posy, q1 = my_client.bot_to_map(posx, posy, None, frame="mrt/camera_link")
                if posx is None:
                    rospy.loginfo(
                        "AR Tag detected but not found in LIDAR. Check width/error/arrow detection"
                    )
                    found = False

                else:
                    q = (0,0,0,1,)  # (0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                    # TODO Add a check if arrow found is nearly in the direction of the previous arrow(or add a warning if it is)
                    # posx, posy, q = my_client.bot_to_map(posx, posy, q, frame="mrt/camera_link")  # map frame
                    # rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))
                    # move to final positions using AR Tag
                    my_client.add_vert_arrow(posx, posy, q)
                    _,__,q=my_client.bot_to_map(0,0,q=None,frame='mrt/camera_link')
                    success = my_client.move_to_goal(
                        *my_client.find_off_goal(
                            posx, posy, q=q,frame="map", offset=(-0.5, 0, 0, 0) #chec offset needed
                        )
                    )
                    print('c1')
                    if success == True:
                        # my_client.add_arrow(*my_client.bot_to_map(posx, posy, q), color=(0,1,1))
                        # prev_x, prev_y, prev_q = posx, posy, q#map frame
                        # my_client.add_arrow(prev_x, prev_y, prev_q, (1,0,1))
                        # my_client.add_to_completed(posx, posy, q)
                        rospy.loginfo("Reached Post")
                        # Flash Green LED
                        my_client.flash_green()
                        return True
                    else:
                        rospy.loginfo("Failed goal: " + str((posx, posy, q)))
                        return False  # TODO

            # elif found == 2:
                # Gate post
                # rospy.loginfo("Gate Post Traversal")
                # # posx1, posy1 = my_client.find_obs_lidar(theta[0])
                # # posx2, posy2 = my_client.find_obs_lidar(theta[1])
                # posx1, posy1 = pts[0][0], pts[0][1]
                # posx2, posy2 = pts[1][0], pts[1][1]
                # if posx1 is None or posx2 is None:
                #     rospy.loginfo(
                #         "AR Tag detected but not found in LIDAR. Check width/error/arrow detection"
                #     )
                #     found = False

                # else:
                #     q = (
                #         0,
                #         0,
                #         0,
                #         1,
                #     )  # (0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                #     # TODO Add a check if arrow found is nearly in the direction of the previous arrow(or add a warning if it is)
                #     posx1, posy1, q = my_client.bot_to_map(posx1, posy1, q, frame="mrt/camera_link")  # map frame
                #     posx2, posy2, q = my_client.bot_to_map(posx2, posy2, q, frame="mrt/camera_link")  # map frame

                #     assert abs(norm([posx1 - posx2, posy1 - posy2]) - 2) <= 0.5

                #     # rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))

                #     # move to final positions using AR Tag
                #     posx = (posx1 + posx2) / 2.0
                #     posy = (posy1 + posy2) / 2.0

                #     success = my_client.move_to_goal(posx, posy, q=q, frame="map")
                #     if success == True:
                #         # my_client.add_arrow(*my_client.bot_to_map(posx, posy, q), color=(0,1,1))
                #         # prev_x, prev_y, prev_q = posx, posy, q#map frame
                #         # my_client.add_arrow(prev_x, prev_y, prev_q, (1,0,1))
                #         # my_client.add_to_completed(posx, posy, q)
                #         rospy.loginfo("Reached Post")
                #         return True
                #         # Flash Green LED
                #     else:
                #         rospy.loginfo("Failed goal: " + str((posx, posy, q)))
                #         return False

            else:
                goal_list=[None,[goal_xy_x+search_offset,goal_xy_y],[goal_xy_x,goal_xy_y+search_offset],[goal_xy_x-search_offset,goal_xy_y],[goal_xy_x,goal_xy_y-search_offset],[goal_xy_x,goal_xy_y]]
                if counter==0:
                    pass  # Sleep for 1-2s and let the bot move towards the goal
                # curr_x,curr_y=my_client.gps2xy(lat,lon)
                elif counter>=1 and counter<=5:
                    my_client.move_to_goal(goal_list[counter][0],goal_list[counter][1])
                elif counter>5:
                    print('AR not found')
                    return False
                found, pts=my_client.urc_recovery_oscillate(type=1,iterations=9,angle=0.4)
                print(counter)
                counter+=1
    elif type==2:
        #Gate post
        rospy.loginfo("Gate Post Traversal")
        posx1,posy1,q1=None,None,None
        posx2,posy2,q2=None,None,None
        initial_found=True
        counter=0
        while not rospy.is_shutdown():
            # AR Tag Detection
            if counter==0:
                _,_,q = my_client.bot_to_map(0,0,None)
                my_client.move_to_goal(*my_client.gps2xy(lat,lon),q=q)
                print('c6')
                rospy.sleep(2)
            if initial_found:
                found,pts = my_client.ar_detect()
                print('initial_found',found,pts)  # theta and orient wrt forward
            if found==2:
                if initial_found:
                    posx1, posy1 = pts[0][0], pts[0][1]
                    posx2, posy2 = pts[1][0], pts[1][1]
                    # posx1, posy1, q1 = my_client.bot_to_map(posx1, posy1, q, frame="mrt/camera_link")  # map frame
                    # posx2, posy2, q2 = my_client.bot_to_map(posx2, posy2, q, frame="mrt/camera_link")  # map frame

                if posx1 is None or posx2 is None:
                    rospy.loginfo("AR Tag detected but not found in LIDAR. Check width/error/arrow detection")
                    found = False
                else:
                    q = (0,0,0,1)  # (0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                    # TODO Add a check if arrow found is nearly in the direction of the previous arrow(or add a warning if it is)
                    print('goals',posx1, posy1, q1,posx2, posy2, q2)
                    # assert abs(norm([posx1 - posx2, posy1 - posy2]) - 2) <= 0.5

                            # rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))

                            # move to final positions using AR Tag
                    c_x,c_y,q=my_client.bot_to_map(0,0,q) #current bot
                    vec=[posx1-c_x,posy1-c_y]
                    perp=[-(posy2-posy1)/(posx2-posx1),1]
                    if np.dot(vec,perp)<0:
                        perp[0],perp[1]=-perp[0],-perp[1]
                    q=functions.q_from_vector3D([perp[0],perp[1],0])
                    posx = (posx1 + posx2) / 2.0
                    posy = (posy1 + posy2) / 2.0
                    # my_client.add_arrow(posx1, posy1, q, color=(0,0.5,0.5))
                    # my_client.add_arrow(posx2, posy2, q, color=(0,0.5,0.5))
                    my_client.add_vert_arrow(posx1, posy1, q)
                    my_client.add_vert_arrow(posx2, posy2, q)

                    success=my_client.move_to_goal(*my_client.find_xy_off_goal(posx, posy, q=q, frame="map",off_dist=0,ahead=-4))
                    # print('c7')
                    #success = my_client.move_to_goal(posx, posy, q=q, frame="map")
                    #print('c8')
                    if success == True:
                        found,pts = my_client.ar_detect()
                        if found==2:
                            my_client.add_vert_arrow(pts[0][0],pts[0][1],q,color=(1,0,0))
                            my_client.add_vert_arrow(pts[1][0],pts[1][1],q,color=(1,0,0))
                            posx=((pts[0][0]+pts[1][0])/2 + posx)/2
                            posy=((pts[0][1]+pts[1][1])/2 + posy)/2
                            success_2=my_client.move_to_goal(*my_client.find_xy_off_goal(posx, posy, q=q, frame="map",off_dist=0,ahead=0.5))
                                # my_client.add_arrow(*my_client.bot_to_map(posx, posy, q), color=(0,1,1))
                                # prev_x, prev_y, prev_q = posx, posy, q#map frame
                                # my_client.add_arrow(prev_x, prev_y, prev_q, (1,0,1))
                                # my_client.add_to_completed(posx, posy, q)
                            # print('c9')
                            if success_2:
                                rospy.loginfo("Reached Post")
                                # Flash Green LED
                                my_client.flash_green()
                                my_client.move_to_goal(*my_client.find_xy_off_goal(posx, posy, q=q, frame="map",off_dist=0,ahead=2))
                                print('c10')
                                return True
                            else:
                                rospy.loginfo("Failed goal: " + str((posx, posy, q)))
                                return False
                        else:
                            my_client.urc_recovery_oscillate(type=2,full_rotation=False)
                    else:
                        rospy.loginfo("Failed goal: " + str((posx, posy, q)))
                        return False
            if found == 1:
                # posx, posy = my_client.find_obs_lidar(theta[0])
                if initial_found:
                    posx1, posy1 = pts[0][0],pts[0][1]
                if posx1 is None:
                    rospy.loginfo("AR Tag detected but not found in LIDAR. Check width/error/arrow detection")
                    found = False
                else:
                    q = (0,0,0,1)  # (0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
                    # TODO Add a check if arrow found is nearly in the direction of the previous arrow(or add a warning if it is)
                    # posx1, posy1, q1 = my_client.bot_to_map(posx1, posy1, q, frame="mrt/camera_link")  # map frame
                    # rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))
                    # move to final positions using AR Tag
                    located=(posx1,posy1)
                    counter1=0
                    while counter1<6:
                        goal_list=[None,[goal_xy_x+search_offset,goal_xy_y],[goal_xy_x,goal_xy_y+search_offset],[goal_xy_x-search_offset,goal_xy_y],[goal_xy_x,goal_xy_y-search_offset],[goal_xy_x,goal_xy_y]]
                        if counter1==0:
                            pass
                        elif counter1>=1 and counter1<=5:
                            my_client.move_to_goal(goal_list[counter1][0],goal_list[counter1][1])
                        elif counter1>5:
                            print('AR not found')
                        found, pts=my_client.urc_recovery_oscillate(type=2,iterations=9,angle=0.4)
                        # posx, posy, q = my_client.bot_to_map(pts[0], pts[1], q=None, frame="mrt/camera_link")
                        if pts is not None:
                            posx2,posy2=pts[0][0],pts[0][1]
                        # posx2, posy2, q2 = my_client.bot_to_map(posx2, posy2, q, frame="mrt/camera_link")  # map frame
                        if found==1:
                            if abs(posx2-located[0])<eps and abs(posy2-located[1])<eps:
                                found=1
                                posx2, posy2, q2=None,None,None
                            else:
                                found+=1
                                initial_found=False
                                break
                        if found==2:
                            break
                        print(counter1)
                        counter1+=1

            else:
                goal_list=[None,[goal_xy_x+search_offset,goal_xy_y],[goal_xy_x,goal_xy_y+search_offset],[goal_xy_x-search_offset,goal_xy_y],[goal_xy_x,goal_xy_y-search_offset],[goal_xy_x,goal_xy_y]]
                if counter==0:
                    print('completed counter 0')
                    rospy.sleep(1)  # Sleep for 1-2s and let the bot move towards the goal
                # curr_x,curr_y=my_client.gps2xy(lat,lon)
                # curr_x,curr_y=0,0
                elif counter>=1 and counter<=5:
                    my_client.move_to_goal(goal_list[counter][0],goal_list[counter][1])
                elif counter>5:
                    print('AR not found')
                    break
                found, pts=my_client.urc_recovery_oscillate(type=2,iterations=9,angle=0.4)
                if found==1:
                    posx1,posy1=pts[0][0],pts[0][1]
                    # posx1, posy1, q1 = my_client.bot_to_map(posx1, posy1, q=None, frame="mrt/camera_link")  # map frame
                    initial_found=False
                if found==2:
                    print('found2 ',pts)
                    posx1,posy1=pts[0][0],pts[0][1]
                    posx2,posy2=pts[1][0],pts[1][1]
                    # posx1, posy1, q1 = my_client.bot_to_map(posx1, posy1, q=None, frame="mrt/camera_link")  # map frame
                    # posx2, posy2, q2 = my_client.bot_to_map(posx2, posy2, q=None, frame="mrt/camera_link")  # map frame
                    initial_found=False
                print('counter',counter)
                counter+=1

    # while not rospy.is_shutdown():
    #     if len(my_client.completed_list) == 5:
    #         rospy.loginfo("End Goal found")
    #         success = my_client.move_to_goal(12,6,q=(0,0,1/np.sqrt(2),1/np.sqrt(2)))
    #         if success:
    #             rospy.loginfo("Completed all goals")
    #         else:
    #             rospy.loginfo("Failed")
    #         break
    #     found, theta, orient = my_client.arrow_detect()#theta and orient wrt forward direction, in degree
    #     if found:
    #         posx,posy = my_client.find_obs_lidar(theta)
    #         if posx is None:
    #             rospy.loginfo("Arrow detected but not found in LIDAR. Check width/error/arrow detection")
    #             found = False

    #         else:
    #             q=(0,0,np.sin(np.pi * orient/(2*180)), np.cos(np.pi * orient/(2*180)))
    #             #TODO Add a check if arrow found is nearly in the direction of the previous arrow(or add a warning if it is)
    #             posx, posy, q = my_client.bot_to_map(posx, posy, q)#map frame
    #             if my_client.is_complete(posx,posy,q):
    #                 rospy.loginfo("Already visited recently found Goal: " + str([posx,posy]))
    #             else:
    #                 i = 1
    #                 #rospy.loginfo("\n arrow found at (in map frame): \n" + str(my_client.bot_to_map(posx, posy, q)))
    #                 my_client.add_arrow(posx, posy, q, color=(0,1,0))#Add Rviz arrow marker, map frame
    #                 success = my_client.move_to_off_goal(posx,posy, q = q, frame = "map", off_dist = 1)
    #                 if success == True:
    #                     #my_client.add_arrow(*my_client.bot_to_map(posx, posy, q), color=(0,1,1))
    #                     prev_x, prev_y, prev_q = posx, posy, q#map frame
    #                     #my_client.add_arrow(prev_x, prev_y, prev_q, (1,0,1))
    #                     my_client.add_to_completed(posx, posy, q)
    #                 else:
    #                     rospy.loginfo("Failed goal: " + str((posx, posy, q)))
    #     if not found:
    #         nearby_goal = just_ahead(prev_x,prev_y, prev_q, off_dist= 0.5 + 0.6*i)
    #         my_client.send_goal(*nearby_goal,frame="map")
    #         rospy.sleep(1)#Sleep for 1-2s and let the bot move towards the goal
    #         i+=1
    #     rate.sleep()

    # Close down the video stream when done


if __name__ == "__main__":
    try:
        main_node()
    except rospy.ROSInterruptException:
        print("Exiting... ")
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise
