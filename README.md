# COMP4034 ARS Project

# Table of Contents

- catkin_ws
  - The main workspace for the project
- frontier_exploration
  - The primary package including nodes and launch files for the project
- waffle_tf_listener.py 
  - Python transformation listener file to display the current position of the robots
    base_footprint in reference to the map.
- RvizProjectTwoConfig.rviz
  - setup config file for RviZ, includes robot camera, global/local path markers and markers to display frontiers.
- auto_exploration.py
  - Finds all frontier clusters and their centroids.
  - Subscribes to /map to take an occupancy grid and locate candidates for frontiers (locations where unoccupied known space meets unknown space).
  - Publishes an occupancy map showing the location of each frontier
  - Publishes color coordinated points to display the distinct clusters of segmented frontiers and their centroids.
  - Automatically navigates to goal locations on the map until the entire map is explore ( or at minimum entropy )
- util.py
  - utility function for auto_exploration.py 

# Summary  

The following is instructions on running frontiers based auto-navigation software  

- A frontier is an area where known unoccupied space meets unknown space.  

The goal of this project is to construct a 'smart' navigation system that can
detect frontier locations, segment them into distinct regions, then navigate
/ explore each region in a way that explores the entire map in the least amount of time.  

The majority of the work here is done in auto_exploration.py  



# Running the Files
(always resource, 90% of the time it fixes the error)

In a new terminal run the following

```console
$ roslaunch frontier-exploration turtlebot3_minitask5.launch
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

This should open a Gazebo and RviZ window.

Next, open the included RviZ config file  

```console
frontier-exploration / RvizProkectTwoConfig.rviz
```

The displays tab should contain,  

- Map/local path/ global path from the start up files section 
- frontiers_map  
        - Subscribed to /frontiers_map
        - Displays the occupancy grid for the group of all frontiers
- MarkerArray  
        - Subscribed to /visualization_marker_array  
        - Displays color coordinated dotes representing distinct clusters of frontiers, centroids and targets.
- EnergyMap  
        - Displays the paths found from robot to centroids created by expanding wavefront
        - This should initially be checked 'OFF' because this marker is distracting (but kinda cool)

Next, in a new terminal run,

```console
$ roslaunch frontier-exploration turtlebot3_navigation.launch
```

If this fails errors you may need to resource the terminal

```console
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch frontier-exploration turtlebot3_navigation.launch
```

Next, run the object detection script

```console
$ rosrun frontier-exploration object_detection.py
```

This will launch a window showcasing where the robot is looking and highlighting the potential objects it sees in its path while also marking them onto Rviz when discovered.

Next, run the frontiers identification script

```console
$ rosrun frontier-exploration auto_exploration.py
```

This will begin an automatic navigation software where the robot will  
find regions of unknown space and navigate to them in a way that optimally  
explores the entire map.  

# Extra Commands

See the position of the robot with,

```console
$ rosrun tf tf_echo /map /base_footprint
```

Another way to display the position of the robot is with a listener script in terminal

```console
$ rosrun frontier-exploration waffle_tf_listener.py
```

This will display the current position of the base of the robot with respect to the map.  
We can issue a command for the robot to move somewhere.  
This can be done with the moveActionClient.  
Running  

```console
$ rosrun frontier-exploration moveActionClient.py -x <goal in x> -y <goal in y>
```

This will translate the position of the robots base frame by (x,y) units.  
Here is an example.

```console
$ rosrun frontier-exploration moveActionClient.py -x -1 -y 1
$ rosrun frontier-exploration moveActionClient.py -x -1 -y -1
```


