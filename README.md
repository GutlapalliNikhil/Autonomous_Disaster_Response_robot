
## Final Project Mobile Robotics : Autonomous Disaster Response & Reconnaissance using TurtleBot 3

### Background
In this project, we apply mobile robotics principles to conduct autonomous reconnaissance in a simulated disaster scenario. Our objective is to deploy a TurtleBot3 in an unknown environment filled with AprilTags, which represent victims in the simulation. The TurtleBot must generate a comprehensive map of the area while also identifying and cataloguing all the AprilTags present, including their unique ID numbers and absolute poses relative to the generated map.

[![Video Title](https://img.youtube.com/vi/UkpYMUttDLI/0.jpg)](https://www.youtube.com/watch?v=UkpYMUttDLI)


To achieve this, the TurtleBot is outfitted with a 360-degree LiDAR scanner for localization and mapping, as well as a Raspberry Pi Camera V2 for AprilTag detection. A successful reconnaissance mission will necessitate the implementation of various techniques such as mobile robotic kinematics and sensing, feature extraction, simultaneous localization and mapping (SLAM), and motion planning. For SLAM and motion planning, we employ Cartographer_ROS and explore_lite, respectively. This integration of technology and methods will enable the robot to navigate and analyze the environment efficiently, providing valuable data for potential disaster response efforts.


### Procedure
* Place the robot in a new, unknown, environment.
* Simultaneously:
  * Utilize Cartographer for SLAM to generate a map of the surroundings and monitor the robot's location.
  * Implement frontier exploration and bespoke nodes to navigate the robot autonomously throughout the area.
  * dentify AprilTags and record their global positions.
* Conclude the search once all tags are detected and a comprehensive map of the environment is produced.

### Use requriment.txt for instaling the necessary pkgs and dependencies

### To run our Code

1. Open terminal and establish an SSH connection between the robot and the host PC:
    * `ssh ubuntu@IP_ADDRESS_OF_RASPI_ON_ROBOT`
- Update the `.bashrc` with the IP addresses of both devices. On the robot, the following two lines should be in the `.bashrc`
    * `export ROS_MASTER_URI=http://IP_ADDRESS_OF_REMOTE_PC:11311`
    * `export ROS_HOSTNAME=IP_ADDRESS_OF_RASPI_ON_ROBOT`
- While the following two lines should be in the `.bashrc` of the host PC:
    * `export ROS_MASTER_URI=http://IP_ADDRESS_OF_REMOTE_PC:11311`
    * `export ROS_HOSTNAME=IP_ADDRESS_OF_REMOTE_PC`
- Run turtlebot bringup on the robot.
    * `roslaunch turtlebot3_bringup turtlebot3_robot.launch`

2. Run `roscore` on the host PC in another terminal

3. Run master.launch file to start the robot.
    * `roslaunch bebop_tag_slam master.launch`
- The list of tags' global poses is automatically saved to your working directory under the filename tags\_DATETIME.txt

## Thank you, Enjoy!
