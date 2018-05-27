# **Path Planning**

---

**Path Planning Project**

The goal of this project is to build a path planner that creates smooth, safe trajectories for the car to follow.

[//]: # (Image References)

[image1]: ./images/StateMachine.png "State machine"

## Rubric Points

Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1020/view) individually and describe how I addressed each point in my implementation.  

---


### Algorithm structure

Before addressing the rubric points a short description of the code structure is presented. The algorithm is divided in the following source code files located in `/src`:

`main.cpp` -> Includes the main routine used to read the sensor fusion data and the car position and speed, as well as to deliver the generated trajectory to the simulator. The generation of the trajectory is performed via calls to functions defined in the `PathPlanner` class.

`PathPlanner.cpp` -> Class that includes the functions to generate a safe trajectory. `behavior_planning` implements the state machine to handle lane changes and speed adjustment. `generate_spline_points` creates five points that will be used to define a spline that will set the trajectory points. `generate_trajectory` creates the trajectory points using a mix of previous path points and recently calculated ones.

For the last two functions of `PathPlanner`, the initial implementation provided by the project walkthrough has been used. Additionally, the provided helper functions were also moved to this class.

#### 1. Car drives according to speed limit

The car drives over the whole way at a maximum speed of `49.5 mph`. The limit is set in the state `keepLane`, line `221` in function `behavior_planning` (see section 5 for explanation of different states).

The velocity is reduced to drive behind the front car at a safe distance when a lane change is not safe. This adjustment is performed in the state `prepareChangeLane`, line `481` in function `behavior_planning`. It is implemented if the distance to the front vehicle is lower than `25m`. A small speed interval centered in the speed of the front car is defined, so that the car velocity oscillates between it. In this way it is ensured that the velocity of the front car is constantly measured (line `472`).

#### 2. Max acceleration and jerk are not exceeded

It is achieved to drive smoothly and avoiding to reach the defined maximum acceleration (`10m/s^2`) and jerk (`10m/s^3`) by increasing and decreasing the velocity each cycle with the factors defined in the lines `80-90` of the header `PathPlanner.h` and used in the if conditions of the lines `221` and `481` (`behavior_planning`). The maximum acceleration and jerk noticed during the simulations where `5m/s^2` and `6m/s^3`.

The deceleration is done through a linear function depending on the distance to the front vehicle. The lower the distance, the greater the deceleration. In this way undesired collisions are avoided.

#### 3. Car does not have collisions

Collisions are avoided by keeping the car at a safe distance of `25m` from front vehicle, by implementing the deceleration mechanism in function of the distance to the front vehicle explained in the section 2, and also by ensuring safe lane changes. A lane change is considered as safe if it fulfills these two conditions:

-> There is no car at an interval of `[-10m, 25m]` in the target lane, being `0m` the Frenet s coordinate of the ego (ego taken as reference).

-> The mean speed of the cars located at the intervals `[-20m, -10m]` and `[25m, 35m]` in the target lane, taking ego as reference, is at an interval `[-4m/s + ego_speed, ego_speed + 4m/s]`.

Check of conditions is implemented in lines `254-261`, `285-292`, `313-320`, `346`, `394` and `433` (`behavior_planning`).

#### 4. Car stays in its lane, except for time between changing lanes.

Car is changing lanes during the time it stays in the states `changeLeftLane`, `changeCenterLane` or `changeRightLane`. The rest of time is always on a specific lane.

#### 5. The car is able to change lanes

The car is allowed to make safe change lanes by applying the following state machine (function `behavior_planning`):

![alt text][image1]

The states are described below:

-> `keepLane` (line `194`). Keeps current lane at a maximum velocity of `49.5 mph` and goes to `prepareChangeLane` as soon as it is detected that the distance to the front vehicle is lower than `25m`.

-> `prepareChangeLane` (line `234`). If ego is in left or right lane, checks safe conditions to change to center lane. If ego is in center lane, checks safe conditions to change to left or right lanes (left lane has priority, except for the case when cars are detected in left lane, but not in right). If a change is safe, go to corresponding change state. If no change is safe, adjust velocity to the one of the front car.

-> `changeCenterLane` (line `518`). Measures Frenet d car coordinate. As soon as it is detected that ego is in center lane, goes to `keepLane`.

-> `changeLeftLane` (line `530`). Measures Frenet d car coordinate. As soon as it is detected that ego is in left lane, goes to `keepLane`.

-> `changeRightLane` (line `542`). Measures Frenet d car coordinate. As soon as it is detected that ego is in right lane, goes to `keepLane`.

#### 6. The car is able to drive at least 4.32 miles without incident.

Simulations proved that car is able to drive without incident during the mentioned time and even more. The following video shows the car action during `5.6 miles` (8 minutes):

[![Alt text for your video](https://i.ytimg.com/vi/rt2zTUoGkRQ/hqdefault.jpg?sqp=-oaymwEZCNACELwBSFXyq4qpAwsIARUAAIhCGAFwAQ==&rs=AOn4CLCrDG_jCyZL-lr8-TEH5Y4nnhtHaA)](https://www.youtube.com/watch?v=rt2zTUoGkRQ&t=137s)
