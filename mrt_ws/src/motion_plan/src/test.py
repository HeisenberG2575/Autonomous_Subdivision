#!/usr/bin/env python

from utils import COL, ROW
from a_star_algorithm import a_star_algorithm
import numpy as np
import sys
import rospy
#import tf2_ros#can use tf also, will have to modify code
from tf import transformations
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Point
import itertools #for a global counter
from PIL import Image

#Initialising
#rospy.set_param('~path', '')
pwd = rospy.get_param('/path', '/home/khush/mrt_ws/src/motion_plan/src/')
mapData=OccupancyGrid()
i = itertools.count()
yaw = 0
pos = Point()

def mapCallBack(data):
	global mapData, pos, pwd#,yaw
	mapData=data
	count = i.next()
	print('Got Map')
	# random grid
	#grid = np.random.randint(0,2,(ROW, COL))
	res = mapData.info.resolution
	grid = np.reshape(mapData.data, (mapData.info.height, mapData.info.width))
	grid[grid == -1] = 100#Unknown map is taken as obstacle
	grid = grid/100
	grid = ~grid+2
	

	x = int((pos.x - mapData.info.origin.position.x)/res)
	y = int((pos.y - mapData.info.origin.position.y)/res)
	src = [x,y]
	dest = [x+50,y+50]#arbitrary, may be blocked, may not be valid
	print('Grid shape: ', grid.shape, 'pos', pos.x, pos.y, 'x,y', x, y)
	
	im = Image.fromarray((grid*255).astype(np.uint8))
	im.save(pwd+'iter_log/'+str(count)+'th_iter.png')
	print('im saved')
	

	with open(pwd+'iter_log/' + str(count)+".txt",'w') as f:
		print('file open')
		sys.stdout = f # Change the standard output to the file we created.
		#print("Grid is \n")
		print(grid[src[0], src[1]], grid[dest[0],dest[1]])
		print("from: " + str(src) + " to: " + str(dest) + "\n")
		a_star_algorithm(grid, src, dest)
		sys.stdout = sys.__stdout__
		print('file closed')
	

def clbk_odom(msg):
    global pos,yaw
    
    # position
    pos = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

def test_node():
	rospy.init_node('test', anonymous=True)
	print('Initiating..')
	sub = rospy.Subscriber('/map', OccupancyGrid, mapCallBack)#(map_topic, datatype, callback fn)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	#rate = rospy.Rate(10)
	#while not rospy.is_shutdown():
	#	pass
	#print(min(mapData.data), len(mapData.data))
	#rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	try:
		test_node()
	except rospy.ROSInterruptException:
		print('Exiting... ')
	except:
		print("Unexpected error:", sys.exc_info()[0])
		raise
