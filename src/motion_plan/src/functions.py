#! /usr/bin/env python
import rospy
import rospkg
import numpy as np
import math
from numpy.linalg import norm
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String, Int32
from visualization_msgs.msg import Marker, MarkerArray
from tf import transformations
from tf.transformations import quaternion_multiply
import tf
from nav_msgs.msg import OccupancyGrid
from pcl_arrow_detect import ArrowDetector
from numpy import nan
import cv2
import geonav_transform.geonav_conversions as gc

path = rospkg.RosPack().get_path("motion_plan")
MARKERS_MAX = 50
ROOT_LINK = "root_link"
USE_ROBUST_ARROW_DETECT = 1
MAX_ARROW_DETECT = 3
ARROW_MAX_ITER = 7
CNT_AREA_THRES = 20
eps=1
class client:
    def __init__(self):
        rospy.init_node("goal_client_node")
        rospy.loginfo("goal_client_node init")

        # define a client for to send goal requests to the move_base server through a SimpleActionClient
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.olat,self.olon=19.13322909, 72.91685103 #49.8999997, 8.90000001
        # rospy.Subscriber('/LatLon',Float64MultiArray,self.gps_callback)
        # rospy.wait_for_message("/LatLon", Float64MultiArray, timeout=5)

        # wait for the action server to come up
        while (
            not self.ac.wait_for_server(rospy.Duration.from_sec(2.0))
            and not rospy.is_shutdown()
        ):
            rospy.loginfo("Waiting for the move_base action server to come up")
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            "map", ROOT_LINK, rospy.Time(0), rospy.Duration(10.0)
        )
        self.mapData = OccupancyGrid()  # map
        self.frame = None
        self.lidar_data = None
        rospy.Subscriber("/map", OccupancyGrid, self.mapCallBack)
        rospy.Subscriber("/scan_filtered", LaserScan, self.lidar_callback)
        rospy.Subscriber('/LatLon',Float64MultiArray,self.gps_callback)
        self.arrow_detector = ArrowDetector()
        print("ArrowDetector Launched")
        self.marker_array_pub = rospy.Publisher(
            "/detected_arrow", MarkerArray, queue_size=10
        )
        self.led_pub = rospy.Publisher("/rover/tasks_status", String, queue_size=10)
        self.cam_servo_pub = rospy.Publisher('/rover/auto_cam_angle', Int32, queue_size=20)
        self.marker_array = MarkerArray()
        self.marker_count = 0
        self.completed_list = []
        self.last_good_location = self.bot_to_map(0, 0, (0, 0, 0, 1))
        self.cam_servo_angle = 0
        rospy.Rate(5).sleep()  #
        # rospy.spin()
    def gps_callback(self,data):
        self.olat,self.olon=data.data[0],data.data[1]

    def xy2gps(self, x, y):
        return gc.xy2ll(x,y,self.olat,self.olon)

    def gps2xy(self, lat, lon):
        x,y = gc.ll2xy(lat,lon,self.olat,self.olon)
        #y = -y
        return x, y
    def ar_detect(self):
        found,pts=self.arrow_detector.ar_detect()
        if found==0:
            return found,pts
        # return found,theta,pts
        quat = (0,0,np.sin(np.pi * self.cam_servo_angle / (2 * 180)), np.cos(np.pi * self.cam_servo_angle / (2 * 180)))
        return found,[list(self.bot_to_map(i[0],i[1],q=quat,frame="mrt/camera_link"))+[0] for i in pts]
    def cone_detect(self):
        print(self.arrow_detector.cone_detect())
        found,val,cone_distance = self.arrow_detector.cone_detect()
        if cone_distance==0 or cone_distance==nan:
            q=q_from_vector3D(val)
            return found,q,cone_distance #sending quaternion
        return found,val,cone_distance #sending pos

    def arrow_detect(self, far=True):
        # returns Found(0/1), position(x,y,z), theta(degrees; rover forward=0)
        if USE_ROBUST_ARROW_DETECT:
            found_arr, cnt_area_arr, pos_arr, orient_arr, timestamp_arr = [], [], [], [], []
            count = 0
            for i in range(ARROW_MAX_ITER):
                found, pos, orient, timestamp, cnt_area = self.arrow_detector.arrow_detect(far=far, visualize=False)
                found_arr.append(found)
                pos_arr.append(pos)
                orient_arr.append(orient)
                cnt_area_arr.append(cnt_area)
                timestamp_arr.append(timestamp)
                if found:
                    count += 1
            if count >= MAX_ARROW_DETECT:
                var_arr = [cnt_area_arr[i] for i in range(len(cnt_area_arr))
                           if found_arr[i]]
                if np.var(np.array(var_arr)) > CNT_AREA_THRES:
                    return False, None, None, timestamp
                else:
                    found_arr.reverse()
                    idx = len(found_arr) - found_arr.index(True) - 1
                    print("foundddddddd\n\n\n")
                    return True, pos_arr[idx], orient_arr[idx], timestamp_arr[idx]
            return False, None, None, timestamp
        else:
            return self.arrow_detector.arrow_detect(far=far, visualize=False)[:-1]

    def mapCallBack(self, data):
        self.mapData = data

    # TODO change to ps, q format for all
    def bot_to_map(self, pos_x, pos_y, q, frame=ROOT_LINK, timestamp=None):
        ps = PoseStamped()
        new_ps = PoseStamped()

        # set up the frame parameters
        ps.header.frame_id = frame

        if timestamp == None:
            timestamp = rospy.Time.now()
        ps.header.stamp = timestamp
        ps.pose.position = Point(pos_x, pos_y, 0)
        ps.pose.orientation = recast_quaternion(q)

        success = False
        while not rospy.is_shutdown() and success == False:
            try:
                new_ps = self.listener.transformPose("map", ps)
                success = True
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                success = False
        new_ps.pose.orientation.x = 0  # TODO Check if advisable to do so
        new_ps.pose.orientation.y = 0
        return new_ps.pose.position.x, new_ps.pose.position.y, new_ps.pose.orientation

    def send_goal(self, xGoal, yGoal, q=None, frame="map"):  # frame=ROOT_LINK
        # relative to the bot location
        # quaternion is a 4-tuple/list-x,y,z,w or Quaternion
        goal = MoveBaseGoal()

        if frame == ROOT_LINK:
            xGoal, yGoal, q = self.bot_to_map(xGoal, yGoal, q)
            frame = "map"
        # set up the frame parameters
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
        goal.target_pose.pose.orientation = recast_quaternion(q)

        rospy.loginfo(
            "Sending goal location - [" + str(xGoal) + ", " + str(yGoal) + "] .."
        )
        self.add_arrow(xGoal, yGoal, q)
        self.ac.send_goal(goal)

    def send_goal_gps(self, Lat, Lon, q = None, frame = "map"):
        x, y = self.gps2xy(Lat, Lon)
        self.send_goal(x, y, q, frame)

    def move_to_off_goal_gps(self, Lat, Lon, q=None, frame="map", off_dist=1.5):
        x, y = self.gps2xy(Lat, Lon)
        rospy.loginfo("off goal"+str(x)+str(y))
        return self.move_to_goal(*self.find_off_goal(x,y, q=q, frame=frame, offset=(0,off_dist,0,0)), frame=frame)

    def move_to_goal(self, xGoal, yGoal, q=None, frame="map"):  # frame=ROOT_LINK
        if frame == ROOT_LINK:
            xGoal, yGoal, q = self.bot_to_map(xGoal, yGoal, q)
            frame = "map"
        self.send_goal(xGoal, yGoal, q, frame)

        self.ac.wait_for_result(rospy.Duration(150))

        if self.ac.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

    def find_xy_off_goal(self, xGoal, yGoal, q=None, frame="map", off_dist=0.5,ahead=0.75):
        goal_initial=self.find_off_goal(xGoal, yGoal, q=q, frame=frame, offset=(-0.0, off_dist, 0, 0))
        goal_final=just_ahead(*goal_initial,off_dist=ahead)
        return goal_final
        #print('move to off goal',goal_initial,goal_final)
        # return self.move_to_goal(*goal_final)

    def find_off_goal(self, xGoal, yGoal, q=None, frame="map", offset=(0, 0, 0, 0)):
        if frame == ROOT_LINK:
            xGoal, yGoal, q = self.bot_to_map(xGoal, yGoal, q)
            frame = "map"
        q = uncast_quaternion(q)
        x0, y0, _ = self.bot_to_map(0, 0, (0, 0, 0, 1))  # Bot location in map
        offset = quaternion_multiply(q, offset)
        offset = quaternion_multiply(offset, transformations.quaternion_inverse(q))
        x1, y1 = xGoal + offset[0], yGoal + offset[1]
        cell1 = get_cell_status(self.mapData, [x1, y1])
        x2, y2 = xGoal - offset[0], yGoal - offset[1]
        cell2 = get_cell_status(self.mapData, [x2, y2])
        if cell1 == 0:
            if cell2 == 0:
                x, y = (
                    [x1, y1]
                    if (norm([x1 - x0, y1 - y0]) < norm([x2 - x0, y2 - y0]))
                    else [x2, y2]
                )
            else:
                x, y = x1, y1
        else:
            if cell2 == 0:
                x, y = x2, y2
            else:
                x, y = (
                    [x1, y1]
                    if (norm([x1 - x0, y1 - y0]) < norm([x2 - x0, y2 - y0]))
                    else [x2, y2]
                )
        # rospy.loginfo(str(["x1",x1,"y1",y1,":",cell1,", x2",x2,"y2",y2,":",cell2]))
        return x, y, q

    # def move_towards_goal(self,xGoal,yGoal, frame="map", move_dist=1.5):
    #       if frame == "map":
    #             xGoal,yGoal,_ = self.map_to_bot(xGoal,yGoal,None)
    #             frame = ROOT_LINK
    #       q = uncast_quaternion(q)
    #       offset = transformations.quaternion_multiply(q, (-off_dist,0,0,0))
    #       offset = transformations.quaternion_multiply(offset, transformations.quaternion_inverse(q))
    #       return self.move_to_goal(xGoal+offset[0],yGoal+offset[1],q, frame)

    def flash_green(self):
        self.led_pub.publish("completed")

    def move_cam_servo(self,angle):
        self.cam_servo_pub.publish(angle)

    def add_to_completed(self, pos_x, pos_y, q):
        self.completed_list.append([pos_x, pos_y, q])
        self.last_good_location = self.find_off_goal(
            pos_x, pos_y, q=q, frame="map", offset=(0, 1, 0, 0)
        )
    def add_vert_arrow(self,x,y,q,color=(0,1,0)):
        q_right = (0, -np.sqrt(0.5), 0, np.sqrt(0.5))
        q_right = quaternion_multiply(uncast_quaternion(q), q_right)
        self.add_arrow(x, y, q_right, color=color)

    def is_complete(self, pos_x, pos_y, q):
        for ps in self.completed_list:
            if (
                norm([ps[0] - pos_x, ps[1] - pos_y]) < 0.7 and diff(q, ps[2]) < 5
            ):  # Replace 5 by 0.2 after adding orientation specific things
                return True
        return False

    def cancel_goal(self):
        self.ac.cancel_goal()
        rospy.loginfo("Goal cancelled")

    def recovery(self, far=True):
        rospy.loginfo("Initiating recovery")
        found, pos, orient, timestamp = self.arrow_detect(far)
        j = 0
        while found == False and j < 6:
            x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
            q = uncast_quaternion(q)
            q = quaternion_multiply(q, (0, 0, np.sin(0.1), np.cos(0.1)))
            self.move_to_goal(x, y, q)
            rospy.sleep(1.0)
            found, pos, orient, timestamp = self.arrow_detect(far)
            j += 1
        j = 0
        while found == False and j < 12:
            x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
            q = uncast_quaternion(q)
            q = quaternion_multiply(q, (0, 0, np.sin(-0.1), np.cos(-0.1)))
            self.move_to_goal(x, y, q)
            rospy.sleep(1.0)
            found, pos, orient, timestamp = self.arrow_detect(far)
            j += 1
        if found:
            orient2 = orient + 90 if orient < 0 else orient - 90
            q = (
                0,
                0,
                np.sin(np.pi * orient2 / (2 * 180)),
                np.cos(np.pi * orient2 / (2 * 180)),
            )
            posx, posy, q = self.bot_to_map(pos[0], pos[1], q, timestamp=timestamp)  # map frame
        if found == False or pos is None or self.is_complete(posx, posy, q):
            rospy.loginfo("Failed. Moving to last known good location")
            self.move_to_goal(*self.last_good_location)
            return False, None, None, timestamp
        else:
            return found, pos, orient, timestamp

    def urc_recovery_oscillate(self,type:int=1,iterations:int=3+1+3,angle:float=0.6,move_rover:bool=False,full_rotation:bool=True): #iterations represents counter_clockwise=3 + reset_to_center=1 + clockwise=3 ; angle to move in rad
        found,pts=self.ar_detect()
        j = 1
        discovered=[0,[]]
        rotations = 0
        while j < iterations+1:
            if move_rover:
                x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
                q = uncast_quaternion(q)
                q = quaternion_multiply(q, (0, 0, np.sin(angle/2), np.cos(angle/2)))
                if j%((iterations+1)/2)==0:
                    angle = -angle
                    q = quaternion_multiply(q, (0, 0, np.sin(((iterations+1)/2)*angle/2), np.cos(((iterations+1)/2)*angle/2)))
                self.move_to_goal(x, y, q)
            else:
                if abs(self.cam_servo_angle + int(angle*180/np.pi))<90 :
                        self.cam_servo_angle += int(angle*180/np.pi)
                else:
                    self.cam_servo_angle = int(90*self.cam_servo_angle/self.cam_servo_angle)
                if j%((iterations+1)/2)==0:
                    angle = -angle
                    if abs(self.cam_servo_angle + int(((iterations+1)/2)*angle*180/np.pi))<90 :
                        self.cam_servo_angle += int(((iterations+1)/2)*angle*180/np.pi)
                    else:
                        self.cam_servo_angle = int(90*self.cam_servo_angle/self.cam_servo_angle)
                self.move_cam_servo(self.cam_servo_angle)
            rospy.sleep(1.0)
            found, pts = self.ar_detect()
            print(found, pts)
            print('recovery stage ',j)
            j += 1
            if found==type:
                self.cam_servo_angle = 0
                self.move_cam_servo(self.cam_servo_angle)
                break
            if found==1 and type==2:
                if discovered[0]==1:
                    comp_x,comp_y=pts[0][0],pts[0][1]
                    if abs(comp_x-discovered[1][0][0])<eps and abs(comp_y-discovered[1][0][1])<eps:
                        pass
                    else:
                        discovered[0]+=1    
                        discovered[1].append([pts[0][0],pts[0][1]])
                if discovered[0]==2:
                    print('found in urc_recovery_final_stage',discovered[0],discovered[1])
                    return discovered[0],discovered[1]
                else:
                    discovered[0]+=1
                    discovered[1].append([pts[0][0],pts[0][1]])    

            if full_rotation and rotations==0:
                j=1
                self.cam_servo_angle = 0
                self.move_cam_servo(self.cam_servo_angle)
                x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
                q = uncast_quaternion(q)
                q = quaternion_multiply(q, (0, 0, 1, 0))
                self.move_to_goal(x, y, q)
                
        # while found == False and j < 12:
        #     x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
        #     q = uncast_quaternion(q)
        #     q = quaternion_multiply(q, (0, 0, np.sin(-0.2), np.cos(-0.2)))
        #     self.move_to_goal(x, y, q)
        #     rospy.sleep(1.0)
        #     found, theta, pts = self.ar_detect()
        #     print(found, theta, pts)
        #     print('recovery stage ',j)
        #     j += 1
        self.cam_servo_angle = 0
        self.move_cam_servo(self.cam_servo_angle)
        if found>0:
            return found, [[i[0],i[1]] for i in pts]
        else:
            return 0, None



    def urc_recovery(self,type=1,move_by_rad=0.6,move_rover=True):
        rospy.loginfo("Initiating recovery")
        found, pts = self.ar_detect()
        j = 0
        discovered=[0,[]]
        while j < 11:
            if move_rover:
                x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
                q = uncast_quaternion(q)
                q = quaternion_multiply(q, (0, 0, -np.sin(move_by_rad/2), np.cos(move_by_rad/2)))
                self.move_to_goal(x, y, q)
            else:
                self.cam_servo_angle += int(move_by_rad*180/np.pi) % 360
                self.move_cam_servo(self.cam_servo_angle)
            rospy.sleep(1.0)
            found, pts = self.ar_detect()
            print(found, pts)
            print('recovery stage ',j)
            j += 1
            if found==type:
                self.cam_servo_angle = 0
                self.move_cam_servo(self.cam_servo_angle)
                break
            if found==1 and type==2:
                if discovered[0]==1:
                    comp_x,comp_y=pts[0][0],pts[0][1]
                    if abs(comp_x-discovered[1][0][0])<eps and abs(comp_y-discovered[1][0][1])<eps:
                        pass
                    else:
                        discovered[0]+=1
                        
                        # print('conv',pts,list(self.bot_to_map(pts[0][0],pts[0][1],q=None,frame="mrt/camera_link")))
                        discovered[1].append([pts[0][0],pts[0][1]])
                if discovered[0]==2:
                    print('rec return',discovered[0],discovered[1])
                    return discovered[0],discovered[1]
                else:
                    discovered[0]+=1
                    # print('conv',pts,list(self.bot_to_map(pts[0][0],pts[0][1],q=None,frame="mrt/camera_link")))
                    discovered[1].append([pts[0][0],pts[0][1]])

        j = 0
        # while found == False and j < 12:
        #     x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
        #     q = uncast_quaternion(q)
        #     q = quaternion_multiply(q, (0, 0, np.sin(-0.2), np.cos(-0.2)))
        #     self.move_to_goal(x, y, q)
        #     rospy.sleep(1.0)
        #     found, theta, pts = self.ar_detect()
        #     print(found, theta, pts)
        #     print('recovery stage ',j)
        #     j += 1
        self.cam_servo_angle = 0
        self.move_cam_servo(self.cam_servo_angle)
        if found>0:
            return found, [[i[0],i[1]] for i in pts]
        else:
            return 0, None


    def add_arrow(
        self, pos_x, pos_y, q, color=(0.2, 0.5, 1.0), pos_z=0
    ):  # color = (r,g,b), in [0,1]
        marker = make_arrow_marker(
            Pose(Point(pos_x, pos_y, pos_z), recast_quaternion(q)),
            self.marker_count,
            color,
        )

        # We add the new marker to the MarkerArray, removing the oldest
        # marker from it when necessary
        if self.marker_count > MARKERS_MAX:
            self.marker_array.markers.pop(0)

        self.marker_array.markers.append(marker)

        # Renumber the marker IDs
        id = 0
        for m in self.marker_array.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        self.marker_array_pub.publish(self.marker_array)

        self.marker_count += 1

    def lidar_callback(self, data):
        self.lidar_data = data

    def find_obs_lidar(
        self, theta, error=30, max_width=0.5, max_depth=0.4, sensitivity=0.2
    ):
        if theta is None:
            return None, None
        # Returns posx,posy for the obstacle found around theta with width less than max_width, depth less than given depth
        obs_list = []  # obstacle list
        theta_idx = int((theta + 90) * 4)
        prev_val = 10
        curr_obs_start = None
        min_diff = error
        arrow = None
        for i in range(theta_idx - error, theta_idx + error):
            if i < 0:
                continue
            if i >= 720:
                break
            if abs(self.lidar_data.ranges[i] - prev_val) < sensitivity:
                if curr_obs_start is None:
                    curr_obs_start = i
            else:
                if curr_obs_start is not None:
                    obs_list.append((curr_obs_start, i))
                    curr_obs_start = None
            prev_val = self.lidar_data.ranges[i]
        for obs in obs_list:
            # Check for width and depth
            width = (
                self.lidar_data.ranges[int(np.average(obs))]
                * self.lidar_data.angle_increment
                * (obs[1] - obs[0])
            )
            depth = max(self.lidar_data.ranges[obs[0] : obs[1]]) - min(
                self.lidar_data.ranges[obs[0] : obs[1]]
            )
            if width > max_width or depth > max_depth:
                continue
            else:
                if min_diff > abs(theta_idx - np.average(obs)):
                    min_diff = abs(theta_idx - np.average(obs))
                    arrow = obs
        rospy.loginfo(str(obs_list))
        if arrow is None:
            return None, None
        r, theta = (
            self.lidar_data.ranges[int(np.average(arrow))],
            np.average(arrow) / 4 - 90,
        )
        return self.bot_to_map(
            r * np.cos(np.pi * theta / 180),
            r * np.sin(np.pi * theta / 180),
            (0, 0, 0, 1),
        )[:2]


def recast_quaternion(quaternion):
    if quaternion is None:
        q = Quaternion(0, 0, 0, 1)
    elif (
        isinstance(quaternion, list)
        or isinstance(quaternion, tuple)
        or isinstance(quaternion, np.ndarray)
    ):
        q = Quaternion(*quaternion)
    elif isinstance(quaternion, Quaternion):
        q = quaternion
    else:
        print("Quaternion in incorrect format: ", type(quaternion))
        q = Quaternion(0, 0, 0, 1)
    return q


def uncast_quaternion(quaternion):
    if quaternion is None:
        q = (0, 0, 0, 1)
    elif (
        isinstance(quaternion, list)
        or isinstance(quaternion, tuple)
        or isinstance(quaternion, np.ndarray)
    ):
        q = tuple(quaternion)
    elif isinstance(quaternion, Quaternion):
        q = quaternion
        q = (q.x, q.y, q.z, q.w)  # lazy+readable code
    else:
        print("Quaternion in incorrect format: ", type(quaternion))
        q = (0, 0, 0, 1)
    return q


def q_from_vector3D(point):
    # http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    q = Quaternion()
    # calculating the half-way vector.
    u = [1, 0, 0]
    norm = np.linalg.norm(point)
    v = np.asarray(point) / norm
    if np.all(u == v):
        q.w = 1
        q.x = 0
        q.y = 0
        q.z = 0
    elif np.all(u == -v):
        q.w = 0
        q.x = 0
        q.y = 0
        q.z = 1
    else:
        half = [u[0] + v[0], u[1] + v[1], u[2] + v[2]]
        q.w = np.dot(u, half)
        temp = np.cross(u, half)
        q.x = temp[0]
        q.y = temp[1]
        q.z = temp[2]
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm == 0:
        norm = 1
    q.x /= norm
    q.y /= norm
    q.z /= norm
    q.w /= norm
    return q


def diff(q1, q2):  # accepts tuples
    q1 = uncast_quaternion(q1)
    q2 = uncast_quaternion(q2)
    q1_inv = transformations.quaternion_inverse(q1)
    diff_q = quaternion_multiply(q2, q1_inv)
    return abs(
        transformations.euler_from_quaternion(diff_q)[2]
    )  # return yaw between two angles (quaternions)


def get_cell_status(mapData, pt):
    # returns grid value at point "pt"- shape:(2)
    # map data:  100 occupied      -1 unknown       0 free
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data

    index = (np.floor((pt[1] - Xstarty) / resolution) * width) + (
        np.floor((pt[0] - Xstartx) / resolution)
    )
    if np.isnan(index) == True:
        print("Error: index is NaN, check : ", resolution, pt, width, Xstartx, Xstarty)
        return 100

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100


def just_ahead(pos_x, pos_y, q, off_dist=0.5):
    q = recast_quaternion(q)
    offset = quaternion_multiply((q.x, q.y, q.z, q.w), (off_dist, 0, 0, 0))
    offset = quaternion_multiply(
        offset, transformations.quaternion_inverse((q.x, q.y, q.z, q.w))
    )
    x, y = pos_x + offset[0], pos_y + offset[1]
    return x, y, q


def make_arrow_marker(ps, id, color=(0.2, 0.5, 1.0)):
    # make a visualization marker array for the occupancy grid
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.ns = "arrows"
    m.id = id
    m.type = Marker.ARROW
    m.pose = ps
    m.scale = Point(1, 0.1, 0.1)
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = 1

    return m

def main():
    sin45 = 1 / np.sqrt(2)
    my_client = client()
    my_client.move_to_off_goal(4, 0, q=(0, 0, sin45, sin45))  # 4,0
    my_client.move_to_off_goal(4, 6)  # 4,6
    my_client.move_to_off_goal(8, 6, q=(0, 0, sin45, -sin45))  # 8,6
    my_client.move_to_off_goal(8, 2)  # 8,2
    my_client.move_to_off_goal(
        12, 2, q=(0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8))
    )  # 12,2
    my_client.move_to_off_goal(
        12, 6, q=(0, 0, sin45, sin45)
    )  # 12,6#check for error, if any
    my_client.move_to_off_goal(8, 10, q=(0, 0, 1, 0))  # 8,10
    my_client.move_to_goal(-2, 10, q=(0, 0, 1, 0))  # -2,10

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Closing")
