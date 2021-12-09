
# museum_nav_bot

This ROS package implements SLAM on a 2 wheeled differential drive robot to map an indoor environment. 

[Demo Video](https://youtu.be/jbd2p1llsqA) 

## Installation
1. Build package from source: navigate to the source folder of your catkin workspace and build this package using:
	```
	$ git clone https://github.com/xkzy/museum_nav_bot.git
	$ cd ..
	$ catkin_make
	```
2. Install Required dependencies:
	```
	$ sudo apt-get install ros-noetic-dwa-local-planner
	$ sudo apt-get install ros-noetic-joy
	```

3. Install pocketsphinx with dependencies

	```
	$ sudo apt-get install -y python python-dev python-pip build-essential swig libpulse-dev git
	$ sudo pip install pyaudio
	$ sudo pip install pocketsphinx
	```

### Install ROS

If you are new to ROS (like me), check this [introductory video](https://www.youtube.com/watch?v=9U6GDonGFHw) for ROS installation details

More instructions can be found on [ROS website](http://wiki.ros.org/ROS/Installation)

## Simultaneous Localization And Mapping (SLAM)

The package uses [slam_gmapping](http://wiki.ros.org/slam_gmapping) to map the environment. For the purpose of this demonstration, we use the Gazebo simulation environment to move around the robot. 

![SLAM Screenshot](https://github.com/xkzy/museum_nav_bot/raw/main/screenshots/slam_gmapping_resized.gif)

1. Load the robot in the Gazebo environment. Default model is the museum. You can change this from ```/worlds/mybot.world```. To continue with default model:
	```
	$ roslaunch museum_nav_bot gazebo.launch 
	```
2. Launch the **slam_gmapping** node. This will also start **rviz** where you can visualize the map being created:
	```
	$ roslaunch museum_nav_bot gmapping.launch
	```
3. Move the robot around. If you have a Joystick, use:
	 ```
	 $ roslaunch museum_nav_bot joy_teleop_launch.launch
	 ```
	 OR 
	 teleop using keyboard:
	 ```
	 $ roslaunch museum_nav_bot keyboard_teleop_launch.launch
	 ```
4. Move the robot in your environment till a satisfactory map is created. 
5. Save the map using:
	```
	$ rosrun map_server map_saver -f ~/test_map
	```
6. Copy the map file to ```~/museum_nav_bot/maps/``` directory and edit the .yaml file to match the path. 
	
## Autonomous Navigation
This package uses the [ROS Navigation stack](http://wiki.ros.org/navigation) to autonomously navigate through the map created using gmapping. 
  
0. To use your generated map, edit ```/launch/amcl_move_base.launch``` and add map .yaml location and name to map_server node launch.
1. Load the robot in gazebo environment:
	```
	$ roslaunch museum_nav_bot gazebo.launch 
	```
2. Start the **amcl**, **move_base** and **rviz** nodes:
	```
	$ roslaunch museum_nav_bot amcl_move_base.launch
	```
3. In rviz, click on ***2D Pose Estimate*** and set initial pose estimate of the robot.
4. To move to a goal, click on ***2D Nav Goal*** to set your goal location and pose. 
## Speech reconition Navigation
This package uses the [pocketsphinx-python](https://github.com/cmusphinx/pocketsphinx-python) The script shows how to control robot with English keywords using pocketsphinx
  
0. To use your generated map, edit ```/launch/amcl_move_base.launch``` and add map .yaml location and name to map_server node launch.
1. Load the robot in gazebo environment:
	```
	$ roslaunch museum_nav_bot gazebo.launch 
	```
2. Start the **amcl**, **move_base** and **rviz** nodes:
	```
	$ roslaunch museum_nav_bot amcl_move_base.launch
	```
3. In rviz, click on ***2D Pose Estimate*** and set initial pose estimate of the robot.
4. Start the **soundplay_node**, **execute_control** and **reconizer** nodes:
	```
	$ roslaunch museum_nav_bot speech_command.launch
	```
4. To move to a goal, Speak to the microphone with command like "goto animal zone.".  

##  [Optional] Joystick Configuration 

To make it easier to map environments, I added a joystick_teleop node to control the robot movement using my xbox controller. If you are using some other controller, you can easily map your buttons:

1. Install the ROS [joy](http://wiki.ros.org/joy) package:
	``` $ sudo apt-get install ros-noetic-joy``` 
2. Connect your Jotstick to your machine and check if its detected:
	```	$ ls /dev/input/```
3. If everything worked, your joystick should show up as jsX. In my case, it showed up as js1.
4. Go to ```/launch/joy_teleop_launch.launch``` and edit the dev parameter value to ```/dev/input/jsX```.
5. Open the ```joy_teleop.py``` script in the ```/scripts/``` folder.
6.  Uncomment the print statements in the ```joyCallback()``` function.
7. Save and run the script using:
	```$ roslaunch diff_drive_robot joy_teleop_launch.launch ```
8. You will see 2 arrays corresponding to the axes and buttons of your Joystick. Press each button/stick and find the index of your controls. Change the ```joy_teleop.py``` script with your respective axes.
