# Author: Paul Niemczyk
Oct 2017 <br>
Udacity Term 2 Project 5: MPC Project


The following sections conform to the rubric required for this project.




## Model Overview

This MPC utilizes IPOPT to provide non-linear optimization to drive the car around the track.
There are two objectives: 

1. Speed: if cost function sufficiently minimized, optimize speed
1. Follow planned trajectory: minmize error (Cross Track Error and Psi error) from trajectory center line.

### Variables: State & Actuators

The car's state is maintained by 6 variables in a vector:

* x:  current x location
* y:  current y location
* psi:  current heading angle
* v:  current linear velocity
* cte:  cross-track error, which is the distance of the calculated MPC trajectory from the waypoints (reference) line
* epsi:  error of psi, which is the angular distance of the calculated MPC trajectory angle from the reference angle

(Note: the waypoints are provided by the simulator in ptsx and ptsy.)

The model considers two additional variables, which are the actuator controls. These are the values actually calculated by the solver and returned back to the simulator via "steering_angle" and "throttle" to drive the car.

* delta:  The steering wheel angle, constrained between [-25,25 degrees]
* a:  Throttle actuation, constrained between [-1,1], where -1 is full braking and +1 is full acceleration


### Motion Model

The MPC uses a basic motion model to calculate the trajectory of the car, and compares
that trajectory to the reference line.  The motion equations are as follows:

* x1 = px + v * cos(psi) * dt;       // Motion equations from MPC.cpp 
* y1 = py + v * sin(psi) * dt;  
* psi1 = psi - v * steer_value / Lf * dt;   
* v1 = v + throttle_value * dt;      // Throttle value is proxy for acceleration

As a side note, in the MPC the values of CTE and epsi are also predicted forward to constrain the differences across timesteps (just as the motion models are also constrained) from time t to time t+1:

* fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
* fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);

### Vector for variables

The MPC must calcualate ahead by N points to determine the car's trajectory and optimize 'delta' and 'a'. 

All of the state and actuator variabes are put into one vector, which takes the form:
~ <x, y, psi, v, cte, epsi, delta, a>

Because there are N points forward, the vector actually contains the following number of variables:
* size_t n_vars = N * 6 + (N-1) * 2;
* // N timesteps, 6 state variables; 2 control inputs (delta,a) over N-1 actuations


### Constraints
The solver requires constraints to be set on each of the 6 variables in the set, over N forward time steps. 
The number of constraints are:
* size_t n_constraints = N * 6;

### Bounds
Upper and lower bounds are created for all the variables.
Most of the variables are set to max values, i.e., +/- 1e19.
However, the actuators take on the following bounds:
* 'delta' --> [-25,25 degrees]
* 'a' --> [-1,1]




## Timestep & Duration

Timestep length N and elapsed duration 'dt' are chosen primarily based on the available compute resources.
Solver is numerically intensive, and a solution is required every update cycle.
An ideal objective is to:

1. maximize N -- provide as much forward predictive visibility as possible in the MPC model
1. minimize dt -- provide a smaller time step for more granularity in modeling

I played with these values slightly, but they did not have a major impact on my car's behavior, given that I stayed within a narrow range that seemed acceptable. Given that there was a 100ms delay, I felt that a dt=0.1 was the maximum acceptable value. N=10 seemed reasonable.

My final choices were:
* N = 10
* dt = 0.1

Note: there was an option to force the solver to use a max amount of compute time. I set mine to 0.2s:
* options += "Numeric max_cpu_time          0.2\n";




## Preprocessing

I applied to two following pre-processing routines, for car translation/rotation and to incorporate a delay of 100 ms.


### Car translation/rotation

The YouTube solution at https://www.youtube.com/watch?v=bOQuhpz3YfU&index=5&list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2
provided an ideal approach to dealing with reference coordinates, in which the reference grid was rotated so that the car's location was at the origin (0,0) and the car's heading angle psi was = 0 (horiziontal).

#### Waypoints & polynomial
This involved translating/rotating all the waypoints, which made the polynomial fitting and evaluation functions easier because they didn't need to deal with any vertical lines (i.e., non-functions).

See main.cpp for the translation/rotation calculations.

#### CTE calculation
Under this rotated condition, CTE became just the y value along the reference polynomial, evaluated at modeled position x. That is, y was the distance above (away) from the reference line at position x.


### Delay -- before translation/rotation

In order to incorporate the 100 ms actuator delay, I used the same motion model equations as above to project the car's initial state 100 ms into the future, to compute new state variables:
* x1
* y1
* psi1
* v1  

These new predicted variables became the new initial state -- BEFORE applying the above translation and rotation. Therefore, the model just assumed the car was where I predicted it would be after applying the actuators (i.e., after the 100 ms delay).


## Model Latency

I described the required latency adjustments in the section above.
The MPC controller works well with latency because the MPC uses motion equations to model what the projected trajectory of the car is. 

Actuator delay can be modeled by using the same equations, to determine the car's state AFTER the actuator delay. The MPC works with that predicted state as if there were no delay at all.

Note: With this simulator, we are not given acceleration, just the 'throttle' value. So in the code, I used 'throttle' as a proxy for acceleration for approximately the same effect.



## Tuning

In addition to values above, the following variables can be tuned in the MPC. I kept the reference velocity ref_v=100 for the duration of the experiment.

I followed the following process:

1. All values = 1 --> Car ran off track immediately
1. cte_weight = 1000, other values =1 --> Did slight better, then ran off track
1. epsi_weight = 1000, other values =1 --> A little better, ran off later
1. cte_weight = 5000, epsi_weight =1000, other values =1 --> Stayed on track but jerky!!
1. Changed delta_change_weight to =1000 --> seemed to do a little better!!
1. Tweaked a_change_weight a little but not much to 10



My final tuned values were:

* double ref_v = 100;   								// Reference velocity

* AD<double> cte_weight = 5000.0; 						// CTE weight        
* AD<double> epsi_weight = 1000.0;        				// epsi weight
* AD<double> v_weight = 1.0;              				// v weight
* AD<double> delta_weight = 1.0;          				// delta actuator weight
* AD<double> a_weight = 1.0;              				// a actuator weight
* AD<double> delta_change_weight = 1000.0;   			// delta actuator smoothness weight
* AD<double> a_change_weight = 10.0; 					// a actuator smoothness weight


My final top speed was approximately 90 mph around the track without any issues.

NOTE: I changed ref_v to =200 after I wrote this, and I still made it around the track with a top speed of approx 101 mph.


## Conclusion

My car made it around the track really well! A successful circumnavigation of the track requires:
1. the right MPC tuning, and 
1. appropriate waypoints provided as inputs. 

A fascinating project.


Thank you, <br>
Paul Niemczyk




