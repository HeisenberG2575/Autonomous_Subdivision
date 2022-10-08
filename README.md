# MRT

This workspace contains a gazebo environment and a bot which can be used for testing path planning algorithms. All of this has been tested and built on ROS melodic 

## To use this on your system, you must have the following:
- ROS
- Gazebo
- Various dependencies of ROS such as Rviz, Robot State Publisher. (if you have installed full desktop version of ROS, you already have them by default)

## To run the files on your system:
1. Clone the workspace.
2. Open terminal and move to the repo and run `catkin_make` and `source devel/setup.bash`
3. To open environment in gazebo run command `roslaunch bot_description spawn.launch`
4. To see the laser scan markings in rviz:
- Open a new terminal and run `roslaunch bot_description rviz.launch`. A RviZ window pops up after this, then make the foll changes
- In the toolbar on left, change **Fixed Frame** to odom
- On the bottom left, click on the `Add` button, and select RobotModel, you should be able to see the bot in RviZ now.
- Similarly, add the laserscan by adding LaserScan after clicking on `Add` button. 
- Change the topic under laser scan to /mrt/laser/scan and you should see some more parameters under the laserscan now.
5. To move the bot manually and see the RviZ markings change, in a new tab run command `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` and navigate as commanded.
6. To know how to take readings from laser scanner into the code. See file reading_lidar.py inside the motion_plan package. To run this file, run command `rosrun motion_plan reading_lidar.py`

## Preventing Common Errors:
- run command `source devel/setup.bash` everytime you open a new terminal
- If while running the spawn.launch file you see errors on the terminal, google them most of them are missing dependency errors.
- If spawn.launch doesn't open gazebo and the terminal shows 'Waiting for services...', then open 2 new terminals and run `roscore` and `rosrun gazebo_ros gazebo`. This should work

## File Structure: This workspace has 4 packages.
- **motion_plan** : has the script for lidar reading
- **gazebo_envs** : has the files for various gazebo envs
- **bot_description** : has the files for bot and it's gazebo plugins.
- **teleop** : to enable manual navigation
