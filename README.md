# CarND-Controls-MPC
*Self-Driving Car Engineer Nanodegree Program*

---
[//]: # (Image References)

[image1]: readme_pics/MPC_clip.gif "MPC clip"
[image2]: readme_pics/variables.gif "Kinematic"
[image3]: readme_pics/kinematic_9_150.gif "Kinematic"
[image4]: readme_pics/actuators.gif "Kinematic"
[image5]: readme_pics/error1.gif "Kinematic"
[image6]: readme_pics/error2.gif "Kinematic"
[image7]: readme_pics/error3.gif "Kinematic"
[image8]: readme_pics/constraints.gif "Kinematic"
[image9]: readme_pics/var_latency.gif "Kinematic"
[image10]: readme_pics/kinematic_latency.gif "Kinematic"
[image11]: readme_pics/veering.png "Kinematic"

In this project I implemented a Model Predictive Control (MPC) that drives a car around a track in Udacity's drive simulator. 

![MPC in simulator clip][image1]

At discrete time intervals, I receive from the simulator:
* the current car position, yaw (heading) and speed;
* the current steering angle, and values for throttle and breaks;
* a list of waypoints the car should follow, expressed in a global reference system.
 
Using an MPC controller I determine the optimal **control values** to be applied to the car to follow the given waypoints, i.e. the steering angle, throttle and breaks. Then I send the simulator:
* the computed control values;
* the car trajectory I predicted and optimised, that the simulator displays in green;
* the waypoints, expressed in the car reference system, that the simulator displays in yellow.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: not supported natively; you can use Ubuntu in a VM or Ubuntu bash for Windows, and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone or download this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc [latency]`.

## Running the Program

Run it from command line with

`./mpc [latency]`

where optional parameter `[latency]` is the simulated actuators **latency time**, expressed in milliseconds. If not provided, it defaults to 100 ms. To run the simulation without latency, provide a value of 0. 
  
Start the simulator, choose "Project 5: MPC Controller" from its start screen, and click "SELECT". 
   
## Kinematic Model

At the heart of any MPC there is a process model. In this case, a kinematic model, describing and anticipating how the car moves based on its actuators (steering wheel, accelerator and brakes).

The model I adopted assumes the car moves on a level plane, and describes the state of the car at time `t` with a six components vector:

[comment]: <> "(
(x_t, y_t, v_t, \psi_t, cte_t, \psi e_t)
)"

![MPC in simulator clip][image2]

they are:
* `x` the x car position (referred to its center of gravity);
* `y` the y car position;
* `v` the car velocity;
* `ψ​` the car yaw (or heading), angle between the `x` axis and the car heading, positive when taken counter-clockwise;
* `cte` the cross-track error;
* `ψe` the yaw error.

The MPC determines an optimal path for the car, over a finite time horizon, and the model includes measures of error, i.e. how far the car is from the computed optimal path.

The **cross-track error** is the car distance from the wanted (optimal) position; the **yaw error** is the angular distance of the car yaw from the wanted (optimal) yaw. They are included in the car state to set-up the optimisation problem the MPC solves.

The simulator provides poise estimates and sensor readings at every time step `t`:
* the current car position`(x, y)`;
* the yaw angle `ψ`;
* the velocity `v`; 
* the steering angle, between 0 and 25 degrees on either side;
* the current throttle/break value `a`, between -1 and 1.

It also provides a list of waypoints that define the planned car route.

Unfortunately the simulator doesn't report the acceleration, which may be inferred with approximation from the throttle/break value.

My MPC implementation interpolates the waypoints with a cubic polynomial, and  tries to direct the car to follow it. 


Given a state at time `t`, the kinematic model computes the state at time `t+1` as follows:

[comment]: <> "(
\\x_{t+1} = x_t+v_tcos(\psi_t)\Delta_t \\
y_{t+1} = y_t+v_tsin(\psi_t) \\
\psi_{t+1} = \psi_t + \frac{v_t}{L_f}\delta_t\Delta_t \\
v_{t+1} = v_t+a_t\Delta_t \\
cte_{t+1}=f(x_t)-y_t+v_tsin(\psi e_t)\Delta_t \\
\psi e_{t+1} = \psi_t-\psi des_t+\frac{v_t}{L_f}\delta_t\Delta_t
)"
 
![MPC in simulator clip][image3]

where:
 * `Δt` is the duration of the time interval between `t` and `t+1`;
 * `Lf` is a constant determined experimentally and provided by the simulator developer;
 * `a` is the acceleration, approximated by the throttle/breaks value that the simulator provides;
 * `ψdes` is the desired yaw angle;
 * `f()` is the polynomial interpolation of the waypoints.
 
The model is approximate. It neglects forces acting on the vehicle that a *dynamic* model could consider. Also, it assumes that acceleration and yaw rate are constant between time `t` and time `t+1`. Further below I report the kinematic model used to handle actuators latency, which is further simplified.
 
 At every discrete time step `t`, the MPC determines a set of values for the actuators. They are

[comment]: <> "( 
 (\delta_t, a_t) \text{ for } t=1,...,N-1
)"

![Image][image4]

 where ` δ` is the steering angle in radians, and `a` is the accelarator/breaks value, positive for the accelerator and negative for the brakes. It is assumed one cannot accelerate and break at the same time.

At time `t` the MPC determines the optimum sequence of actuator values to be applied at time `t, t+1, ..., t+N-1`; the identified `N-1` time intervals are of constant duration `Δt`. Only optimum values for time `t` are applied. When the MPC receives another set of measurements from the simulator, and an updated list of waypoints, it resets `t` at that time, and computes yet an optimum sequence of actuator values.
    
To find the optimum actuator values the MPC solves an optimisation problem, given by a cost function, the kinematic model at time steps `t, t+1, ..., t+N-1` and the constraints detailed below. The cost function has the following addends:

[comment]: <> "( 
\sum_{t=0}^{N-1}(w_1cte_t^2+w_2\psi e_t^2) 
)"

![Image][image5]

to reward driving close to the waypoints and the desired yaw angle;

[comment]: <> "( 
\sum_{t=0}^{N-1}w_3(v_t-v_{ref})^2
)"

![Image][image6]

to ensure the car keeps moving, trying to attain the target speed

[comment]: <> "( 
\sum_{t=0}^{N-2}(w_4\delta_t^2+w_5a_t^2)+\sum_{t=0}^{N-3}[w_6(\delta_{t+1}-\delta_t)^2+w_7(a_{t+1}-a_t)^2]
)"

![Image][image7]

to penalise action by the actuators, and to limit changes in actions between adjacient time intervals, for a more comfortable drive and to prevent loosing control of the vehicle.

Weights `w` allow adjusting the relative importance of the different parts contributing to the cost.

Actuators have limitation on the values they can apply, modeled by constraints:

[comment]: <> "( 
\\ \delta_t\in[-25^{\circ}, 25^\circ]\\
a_t\in [-1,1]
)"
   
![Image][image8]

for `t=0, 1, ..., N-1`.

## Polynomial Fitting and MPC Pre-Processing

Before fitting the cubic polynomial to the waypoints, I transform them into the car reference system at time `t`. That is, a coordinates reference system with origin on the car poise, where `x=0` and `y=0` in the car center of gravity, and a yaw of 0 radians is aligned with the `x` axis. In this reference system, the car has coordinates `(0,0,0)`.

This simplifies the kinematic model, also for handling latency (se below), and allows me to send the optimisation results to the simulator without further coordinates transformation. The simulator in fact expects a list of waypoints, and the predicted car positions at time `t, t+1, ..., t+N-1`, in the car reference system (at time `t`). 

## MPC with Latency

My program introduces a user-defined delay before sending the simulator the optimisation results. This is to simulate a latency time in actuators, i.e. a delay between the MPC directing the actuators, and the actuators applying any change.

An MPC, differently from a simple PID controller, is well equipped to handle latency, by incorporating it into the process model. My implementation predicts, at time `t`, what the car state will be at the end of the latency time; the predicted state vector is:

[comment]: <> "( 
(x_t', y_t', v_t', \psi_t', cte_t', \psi e_t')
)"

![Image][image9]

Expressing the state in the car reference system at time `t`, and **assuming constant velocity during the latency time**, the kinematic model gives the simplified state udpate equations:

[comment]: <> "( 
\\x_t'=v_tl\\
y_t'=y_t\\
\psi_t'=-\frac{v_t}{L_f}\delta_tl\\
v_t'=v_t\\
cte_t'=cte_t+v_tsin(\psi e_t)l\\
\psi e_t'=\psi e_t+\psi_t'
)"

![Image][image10]

where `l` is the latency time.

Assuming constant velocity during latency, that is `a` is 0, yields an approximation, and the result is robust enough to to drive the car reliably around the track with a latency of up to 180 ms. Results vary depending on the performance and load of the computer running my program and the simulator, because they impact the controller response time, and potentially introduce further latency. 

## Parameters Tuning

Implementing the MPC I had to choose: 
* `N`, the number of time-steps in the finite-horizon optimisation;
* `Δt`, the time intervening between two consecutive time-steps, taken constant.
 
Product `NΔt` gives the overall time, starting from `t=0`, for which the controller predicts and optimises the car trajectory. A too small `NΔt` doesn't allow the car to stay on track, as the controller doesn't look ahead enough; a too large one makes the computation heavier and doesn't bring actual benefits, as the model is approximate and its prediction accuracy degrades the more it goes into the future.
  
A small `Δt` would be desirable for more frequent, and therefore more accurate, calculation of actuator values. However, with a latency time of 100 ms, I observed that setting `Δt` to 120 ms or lower makes the car behave erratically and go off-track, with a sweet spot around 130 ms.
  
Once set `Δt` to 130 ms, I set `N` to 11 as the smallest value that gave me a predicted trajectory subjectively close to the waypoints interpolation. A value smaller than 11 gives a predicted trajectory that often veers off visibly at its end: see picture below, for `N=9` and `Δt=13 ms` .

![Image][image11]

