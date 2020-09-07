# Path Planning Project Writeup
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./images/path_planning.png "Proof lap completion"
[image2]: ./images/limits.png "limits of implementation"


## [Rubric](https://review.udacity.com/#!/rubrics/1971/view) Points
In the following I will briefly discuss the rubrics for this project.

### Compilation
1. The code compiles correctly.
   * The code compiles without errors with `cmake` and `make`.
   * No changes to the `CMakeList.txt` were necessary. 

### Valid Trajectories
1. The car is able to drive at least 4.32 miles without incident.
   * The car drives according to the speed limit.
   * Max Acceleration and Jerk are not exceeded.
   * Car does not have collisions.
   * The car stays in its lane, except for the time between changing lanes.

2. The car is able to change lanes.
   * The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

![Proof lap completion][image1]


### Reflection
1. There is a reflection on how to generate paths.
   * This is discussed in the follwing Model Documentation


### Model Documentation
I started out by closely following the Project Q&A by David and Aaron. Their clever usage of the [spline header functions](https://kluge.in-chemnitz.de/opensource/spline/) really made it a breeze to get the car moving along the s-axis in the frenet coordinate system. So, I decided to use the provided starter code and only slighty tuned values for acceleration/deceleration and the gap for the spline's anchor points. Having a larger gap for the spline's anchor points yields smoother trajectorys and thus will lead to less jerk and lateral acceleration.

In this part of the code (lines 263 - 406) the trajectory is generated, converted to x and y coordinates (in the world coordinate frame) and finally pushed into the vectors `next_x_vals` and `next_y_vals`. As the leftover trajectory from the last calculation is always provided by the simulator, the code first checks how many points are left. At the start of the simulation this trajectory is empty and we use the starting pose (current position with current yaw) and a pose just one meter back as the first two anchor positions for subsequent calculations (lines 271 - 287). If the trajectory is already filled with some points, we are using the last two of those positions subsequent calculations (lines 289 - 308).

In the next step three more anchor positions, drawn from the map 40 m, 80 m and 120 m ahead of the current ego state in frenet s-coordinate, are added (lines 312 - 327). To simplify further calculations the resulting 5 anchor positions are transformed to the vehicle coordinate system (lines 331 - 337). These anchor points are then used to define a spline `s` (lines 343 - 348). 

The vectors `next_x_vals` and `next_y_vals` which hold the x and y coordinates for the path planner are defined in lines 351 and 352. The left-over trajectory from the last time step are reused and copied to the vectors (lines 354 - 359). We are always aiming for a vector size of 50 (line 374). The remaining spots are filled with points that are interpolated by the spline `s` using the provided 5 anchor points (lines 361 - 406). The spacing between the points is given by the tick rate (20 ms) and the desired velocity (lines 361 - 370 and lines 378 - 387).

To prevent collisions with traffic in front, I also went with Aarons approach to loop through all vehicles, check for their lane and if it is closer than 20 m it will raise a flag `too_close` (lines 117 - 151). Additionally I am saving the lowest speed from any vehicle closer than the 20 m in front of the ego vehicle. Ego speed will be reduced to this value (lines 153 - 161). 

In the same loop I am also already checking for vehicles that are closer than 100 m and raise another flag `check_for_lanechange` (lines 134 - 137). This gives us the opportunity to check for a lanechange when we are approaching a preceding slower moving vehicle in order to overtake it. In lines 163 - 297 three checks for possible lanechanges are done depending on the current ego lane. If we are currently in the middle lane (lane == 1), all vehicles in the left lane and all vehicles in the right lane are checked for their s positions with respect to the ego vehicle. The closest gap to the front and to the back are saved. If a gap 8 m to the back and 55 m to the front exists (all vehicles are transformed to the next timestep using their respective velocities), it is safe to use that lane and we will try to use it to overtake the slow vehicle in front of us (lines 166 - 224). Similarly, the checks are done when we are currently in the left or right lane (lane == 0 or lane == 2). In these cases, only the middle lane must be checked (lines 226 - 260).

Using this straight-forward approach, the vehicle can safely navigate through the traffic in the simulator and complete multiple laps without incident (see image above). Due to it's simplicity it is possible to stage situations where a optimal solution is taken (se image below). In this case the ego vehicle is stuck behind a slow moving vehicle on the left lane and the middle lane is blocked by another vehicle at the same slow speed. In this situation, an experienced driver could purposely slow down even further to fall behind the vehicle in the middle lane and overtake on the far right lane. Advanced techniques such as those can allow the ego vehicle to complete a lap even faster in those special situations. 

![limits of implementation][image2]