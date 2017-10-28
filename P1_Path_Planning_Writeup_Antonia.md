#**Term 3 : Path Planning Project**


**MPC Controller Project**

The goals / steps of this project are the following:

[//]: # (Image References)

[image0]: ./../master/tools/Predictive_Model_Equations.png "model_equations.png"
[image1]: ./../master/tools/initial_position.png "waypoint_rotation.png"

## Rubric Points
Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1020/view) individually and describe how I addressed each point in my implementation.  


### Description
First of all this was the far most complex project I have worked on so far in this Udacity program. I even introduced a proper logging facility in order to be able to have better debugging possibilities.
At the end it is still far from being finished or even polished, but I really ran out of time and didn't want to risk my matrimony - so I stopped in between when I was sure that the criterias of the project rubric are fulfilled.

So I apologize for the code which is sometimes more complex than required, contains two quick hacks and is not refactored. At the end I know it better and would actually re-write a lot of parts - but that's how SW developement is like sometimes...

#### The Behavior Planner (class BehaviorPlanner.cpp)
Now coming to the main criteria of my implementation:
*  I introduced a strict seperation of behavior planning (class BehaviorPlanner in file BehaviorPlanner.cpp) and trajectory generation (classes Trajectory.cpp and TrajectoryWIP.cpp)
* the Behavior planner is recursive and can go into future prediction for maximum 10 steps (1 step = 1 second). This makes planning a bit more strategic, however for a deep strategy more computing power would be required (different thread with a different time). I realized too late that a maximum depth of 4 seconds (see main.cpp in line 118 `planner.predictSituation(4)`) is possible in order to be able to finish that calculations within 20ms
* the Behavior planner works according the to following steps:
  - predict the future with all possible (and reasonable) maneuvrs (BehaviorPlanner.cpp in function `iteratePredictions()`)
  - evaluate each and every situation which is predicted according to cost functions 
  - choose best maneuvr sequence for trajectory calculation (the best maneuvr sequence has the minimal deviation from all maneuvrs) - see main.cpp line 124 `planner.getManeuvrDataForTrajectory(proposedPath)`
  - the possible 5 basic maneuvrs are "do nothing" , "Accelerate", "Decelerate", "Change to left lane, "Change to right lane"
  - so that means that for 5 basic maneuvr and a prediction horizon of 4 seconds already `5⁴ = 3125` situations could evolve in worst case which  all have to be evaluated according to cost functions

* limitations of this Behavior planner approach:
  - it does not work in 20ms for deeper calculations
  - predictions of other vehicles are simple (no lane change currently)
  - no acceleration/deceleration of other vehicles is considered (would make it even more complex!)
  - only 5 basic maneuvrs are considered, but of course also a maneuvr like "emergence braking" would be possible

At the end I feel this approach was highly over-engineered for the given project, but it was fun at the end and I like maneurs like a fast change from the very left lane to the very right lane...

Nevertheless in general I have implemented a safe driving behavior with the ability also to avoid collisions in case of bad driving behavior of the other vehicles. But there are still some problems remaining:
* I have not implemented a reaction to a sudden and very dangerous lane change of a vehicle: the calculated trajectory is being executed and in case of 50 available waypoints it means that the ego vehicle has a reaction time of at least one second
* second "problem" is that my behavior planner tends rather to slow down in dangerous situations
* in case a vehicle is approaching very fast from behind and ignoring a safety distance my behavior planner is getting "nervous" and executing potentially dangerous maneuvrs in order to avoid this dangerous situation of a vehicle ignoring the safety distance. Possibly I should simply reduce the risk evaluation of this...!?

#### Trajectory planner Trajectory.cpp (Default)
This is the trajectory planner which is using the pure spline approach as explained in the Q&A Walkthrough video. Actually I use this algorithm 1:1 and just insert two inputs: 
* target lane
* target speed

Actually there is nothing more to tell about this trajectory planner - it is very well explained in the Q&A session and thats the reason why I wanted to implemented a different approach and use the JMT.


#### Trajectory planner TrajectoryWIP.cpp
The WIP stands for "Work in Progess" as I'm still not really happy with the current status.

In order to use this trajectory planner set the boolean variable `takeSpline = false` (see main.cpp line 133 `bool takeSpline = true;`).

In opposite to the Trajectory.cpp it doesn't construct a proper trajectory point-by-point per design, but uses the JMT class (JMT = Jerk Minimizing Trajectory) which we used in the lessions. This approach is really cool as you don't have to care about the details of acceleration and jerk and just insert the starting point and the end point.
I calculate in Frenet coordinates and took some time until I've found out the ingredients for a successful usage of JMT:
* use the last end point of the JMT as starting point for the extension of the trajectory (in case you use the current vehicle position it is impossible to smoothly stitch the two trajectories together)
* use an improved transformation algorithm from Frenet coordinates to `(x,y)` coordinates (see the method `getXY_JMT()`) which is using a spline interpolation and ensures a smoothness in the transformation. To be honest I found the hint for that approach in the Udacity forum and just implemented it as proposed there


Which problems do I face with this trajectory planner:
* sometimes (really rarely - around once or twice in 4 miles) it happens that I get the message "out of lane" or/and "speed limit violation". I'm quite sure that this is not a problem with my algorithm, but I couldn't really find the time now in order to spot the problem
* at the end of the race course (after `s>6900`) I get a weird behavior of my ego vehicle which is suddenly executing a rectangular lane change or "jump". After initial analysis I found out that it most likely is a problem either with the simulator which provides wrong `d` values or the calculation of the `(x,y)` values by the method `getXY_JMT()` is failing at the last waypoint.
* as I use 100 waypoints for sending back to the simulator I even have a reaction time of 2 seconds in worst case

So finally again it was a great peasure working at that project, and I would highly appreciate further hints in order to improve my second trajectory planner TrajectoryWIP - what can I do in order to avoid the spontaneous "out of lane" messages or the "jump" at the end of the race course?