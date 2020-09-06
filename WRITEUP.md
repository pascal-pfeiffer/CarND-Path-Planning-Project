# Path Planning Project Writeup
Self-Driving Car Engineer Nanodegree Program

In the following I will briefly discuss the rubrics for this project. 

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

