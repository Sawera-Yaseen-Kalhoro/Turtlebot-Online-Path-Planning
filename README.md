# Lab 1: Online Path Planning

Team members:

Sawera Yaseen - u1992455

Vania Katherine Mulia - u1992883


## Running the Package

To run the simulation and the online path planning node altogether, launch `online_path_planner.launch` from the terminal using the command line:

```bash
roslaunch turtlebot_online_path_planning online_path_planner.launch
```

## Overview of the Package
### Path Planner

This part explains the path-planning algorithm implemented.

__Functionality:__
The Rapidly-exploring Random Trees (RRT) algorithm with the path smoothing method is implemented for path planning of a turtle-bot in a 2D space. It is a sampling-based algorithm here used to explore the configuration space of the robot, generating the random configurations and connecting them with the nearest existing nodes of the tree to extend it. To ensure that the path is valid, a state validity checker is used and to smooth the computed path the smooth path function is used.

__Usage:__
To utilize this planner class, it is being initialized with parameters such as state validity checker to check, maximum planning time and dominion.  Then compute path method is used to generate the path from the start configuration to the goal position by ensuring the collision-free path. Lastly, the smooth path method smoothes the generated path.

__Implementation:__
The main implementation of this planner lies in the `compute_path` method where the RRT algorithm is implemented. In this method, the RRT algorithm is iterated until it reaches the maximum time. In every iteration, the steps involve:

* generating a random configuration (`rand_conf` method),  
* identifying the nearest configuration (`nearest_vertex` method), 
* generating a new configuration (`new_conf` method) by moving an incremental distance (`delta_q`) to the random configuration while ensuring the path validity using the state validity checker
* expanding the tree with the new configuration

If the valid path from the start and goal positions is found, it is returned. Furthermore, to refine the generated path, the `smooth_path` method is implemented.

__Considerations:__
In the RRT algorithm, there are several parameters essential to tune like incremental distance (`delta_q`), probability of selecting goal position as random configuration (`p_goal`), and the  maximum time (`max_time`) to balance between both computational efficiency and the generated path quality. `max_time` is passed into the planner when calling the `compute_path` function (which initializes the `Planner` instance), while `delta_q` and `p_goal` can be tuned in the `Planner` class (line 124 of `online_planning.py`). Additionally, it is better to ensure the proper working of the state validity checker to evaluate the random configurations as valid to avoid collisions.

### Controller
This part describes the proportional controller function implemented to move the turtle-bot from the current position to the goal position.

__Functionality:__
The `move_to_point` function is implemented to move the robot from its current position to the goal position using a proportional gain controller. 

__Usage:__
To use this function we need to provide the robot’s current position, goal position, and the proportional gain values for linear velocity (Kv) and angular velocity (Kw). Then the function returns the linear and angular velocities required to move the robot to the target position.

__Implementation:__
In this function initially, the Pythagorean  theorem is implemented to compute the distance between the current position and the goal position.  Then the desired orientation of the turtle-bot towards the goal position is computed using the arctan2 function. The difference between the desired and current orientation is calculated to adjust the angular velocity required to align with the goal position. This alignment within the specified threshold is also used to calculate the linear velocity that moves the robot to the goal position.

__Considerations:__
In this function, we need to carefully select the Proportional controller gain values (Kv, Kw) at controller parameters sections in `turtlebot_online_path_planning_node.py`, so that the turtle-bot balance between both speed and stability. The other thing that needs to be considered in this function is the threshold value for determining the alignment of the robot’s head to the target position.

The controller can be tuned to make the robot follow the path better. Parameters that can be controlled are Kv, Kw, v_max, and w_max in `turtlebot_online_path_planning_node.py`, angle error tolerance in `online_planning.py` (line 338).

## Results

A video of the simulation can be found in this [link](https://drive.google.com/file/d/15FriPiwZPeUYjwzFR94ILeQm_Osz4zmR/view?usp=sharing).

## Issues

__The path generated is not optimum.__ 

This is because RRT is a sampling-based path planner. Sometimes, the path generated seems like a detour path that moves the robot further from the goal instead of closer (see the video at timestamp 2:39). Especially, that detour path is actually not feasible, however it is still valid considering what the robot has known about the map.

A possible way to reduce this occurence is to increase the value of `p_goal` in the planner. This will make the RRT algorithm generates a completely random configuration less frequently, and thus, the path formed has less randomness. However, we also found that this solution comes with a possibility of the planner not being able to compute a path at all within the time limit (`max_time`).

__The planner struggles with narrow pathways.__

Due to the randomness of the RRT algorithm, we have found that the planner struggles to find a path through narrow pathways (see the video at timestamp 8:56). It takes longer to compute the path, and given the time limit of the planner (`max_time`), sometimes the planner fails to compute the path. A possible solution to this is increasing `max_time` of the planner.

__The planner is sensitive to obstacle.__

The planner is sometimes very sensitive to obstacles that it replans when a part of the path is close to a newly-discovered obstacle, even though the path does not seem to go through the obstacle. And when it replans, the path is not optimum (refers to the previous point).

A possible solution for this is to increase the time interval between path checking in `turtlebot_online_path_planning_node.py` line 80. This will make the system check the validity of the path and possibly replan less frequently.

Another possible solution is to decrease the distance threshold for the state validity checker. This is done when initializing the `OnlinePlanner` instance (line 237 of `turtlebot_online_path_planning_node.py`). This will make the state validity checker consider less area for the validity checking. However, this will make some parts of the path very close to obstacles, and if the robot is unable to follow the path perfectly (due to tolerances in the controller), the robot may get stuck in an obstacle (which is a problem that we do not tackle in this program).

__Path checker overshoots an obstacle.__

We have made an adjustment in the `check_path` method in the state validity checker regarding the step size to discretize the path. While it makes sense to set the step size as 2 times the distance threshold, we have found that sometimes the `check_path` method would "overshoot" an obstacle and consider a path valid even though it passes through an obstacle. Thus, we have set this step size to be equal to the distance threshold (line 83 of `online_planning.py`). This will create an overlap during the checking, but it is a safer choice.