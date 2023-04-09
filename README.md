# A-star Algorithm on ROS Turtlebot3
Video Link - https://youtu.be/XunpaM0OAZg <br>
Github Link - https://github.com/whosthemaan/A_Star_Turtlebot3

Both Part 1 and Part 2 are included here together and hence don't need to be run seperately

### Introduction to the Project

In this project, the Astar path planning algorithm was used on a point robot to help it navigate through obstacles.
### Considering the clearance, we have calculated the final coordinates of the vertices of the obstacles.  
### The clearance is also being considered at the walls of the field.

<br>

## **Installation and Running**
1. Download and extract the files.

2. Creating an executable for the main code.

```
cd ~/catkin_ws/src/astar_turtlebot/scripts
chmod +x astar_turtlebot.py
cd ../../../
catkin_make
```
3. Source the catkin workspace and then run the program

```
source ./devel/setup.bash
roslaunch astar_turtlebot project3.launch
```
A second terminal will pop up displaying the following:

```
x coordinate for the start node(in meters, same as the one given in the roslaunch command): 0
y coordinate for the start node(in meters, same as the one given in the roslaunch command): 0
orientation for the start node(in radians, same as the yaw value given in the roslaunch command): 0
x-coordinate of the goal node(in meters): 3.5
y-coordinate of the goal node(in meters): 0.8
Enter the first value of RPM: 100
Enter the second value of RPM: 50
Enter the clearance(Basically maximum distance of the robot from the obstacle given in meters): 0.05
```

After entering all these values in the terminal, the A-star algorithm finds the optimum path between the entered start node and goal node.

### Results
Blue: Optimal Path | Green: Explored | Red: Obstacles <br>
The default inputs are provided in the launch file. <br>
Both the parts are included in a single video
<br>

### Authors
Omkar Chittar - ochittar - 119193556 <br>
Rohan Maan - rmaan - 119061770