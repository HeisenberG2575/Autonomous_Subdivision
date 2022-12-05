# MRT

This workspace contains a gazebo environment and a bot which can be used for testing path planning algorithms. All of this has been tested and built on ROS Noetic

## To use this on your system, you must have the following:
- ROS
- Python3
- Gazebo
- Various dependencies of ROS such as Rviz, Robot State Publisher. (if you have installed full desktop version of ROS, you already have them by default)
- python3-pip
- python-is-python3
Use ros_install.sh script if you haven't installed these.

## To run the files on your system:
1. Clone the workspace 
```bash
git clone --recurse-submodules https://github.com/iitbmartian/Autonomous_Subdivision.git
# git clone --recurse-submodules git@github.com:iitbmartian/Autonomous_Subdivision.git
## incase you forgot to clone submodules recursively, do the following 
# git submodule update --init --recursive

## incase you haven't installed ROS
# bash ./Autonomous_Subdivision/ros_install.sh
```
3. Open terminal and cd to the repo, install dependencies and build the ws (run catkin_make)
```bash
cd Autonomous_Subdivision
## This may take time, also requires sudo access
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
catkin_make
```
5. source the ws and launch the simulation
```bash
source devel/setup.bash
# source devel/setup.zsh
roslaunch motion_plan autonomous.launch
```
Use this for headless(no gazebo gui)
```bash
roslaunch motion_plan autonomous.launch gazebo_gui:=false
```


## Preventing Common Errors:
- run command `source devel/setup.bash` everytime you open a new terminal
- If while running the spawn.launch file you see errors on the terminal, google them most of them are missing dependency errors.
- If spawn.launch doesn't open gazebo and the terminal shows 'Waiting for services...', then open 2 new terminals and run `roscore` and `rosrun gazebo_ros gazebo`. This should work

## File Structure: This workspace has 4 packages.
- **motion_plan** : has the script for lidar reading
- **gazebo_envs** : has the files for various gazebo envs
- **bot_description** : has the files for bot and it's gazebo plugins.
- **teleop** : to enable manual navigation
