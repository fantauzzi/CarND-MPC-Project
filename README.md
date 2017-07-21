# CarND-Controls-MPC
*Self-Driving Car Engineer Nanodegree Program*

---
[//]: # (Image References)

[image1]: MPC_clip.gif "MPC clip"

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

The model I adopted assumes the car moves on a level plane, and describes the state of the car at time `t` as a six components vector:

??? fomula here

they are:
* `x` the x car position (referred to its center of gravity);
* `y` the y car position;
* `v` the car velocity;
* `ψ​` the car yaw (or heading), angle between the `x` axis and the car heading, positive when taken counter-clockwise;
* `cte` the cross-track error;
* `ψe` the yaw error.

The MPC determines an optimal path for the car, over a finite time horizon, and the model includes measures of error, i.e. how far the car is from the computed optimal path.

The **cross-track error** is distance of the car (its center of gravity) from the wanted (optimal) position; the **yaw error** is the angular distance of the car yaw from the wanted (optimal) yaw. They are included in the car state to set-up the optimisation problem the MPC solves.

Given a state at time `t`, the kinematic model computes the state at a future time `t+1` as follows:

??? formula here

The model is approximate. It neglects forces acting on the vehicle that a *dynamic* model could consider. Also, it assumes that acceleration and yaw rate are constant between time `t` and time `t+1`. Further below I report the kinematic model used to handle actuators latency, which is further simplified.
 
 At every discrete time step `t`, the MPC determines a set of values for the actuators. They are
 
 ??? formula here
 
 where ` δ` is the steering angle, and `a` is the accelarator value if positive, the breaks value if negative. It is assumed one cannot accelerate and break at the same time.
 
The simulator provides poise estimates and sensor readings at every time step `t`:
* the current car position`(x, y)`;
* the yaw angle `ψ`;
* the velocity `v`; 
* the steering angle, between 0 and 25 degrees on either side;
* the current throttle/break value, between -1 and 1.

Unfortunately the simulator doesn't report the acceleration, which may be inferred with approximation from the throttle/break value.

My MPC implementation interpolates the waypoints with a cubic polynomial, and  tries to direct the car to follow it. 

At time `t` the MPC determines the optimum sequence of actuator values to be applied at time `t, t+1, ..., t+N-1`, see ??? ref here; the identified `N-1` time intervals are of constant duration `Δt`. Only optimum values for time `t` are applied. When the MPC receives another set of measurements from the simulator, and an updated list of waypoints, it resets `t` at that time, and computes yet an optimum sequence of actuator values.
    
To finding the optimum actuator values the MPC solves an optimisation problem, given by a cost function, the kinematic model at time steps `t, t+1, ..., t+N-1` and the constraints detailed below. The cost function has the following addends:

??? formula here

to reward driving close to the waypoints and the desired yaw angle;

??? formula

to ensure the car keeps moving, trying to attain the target speed

??? formula

to penalise action by the actuators, and to limit changes in actions between 
adjacient time intervals, for a more comfortable drive and to prevent loosing control of the car.

Actuators have limitation on the values they can apply, as modeled by the constraints:
   
??? formula

## Polynomial Fitting and MPC Pre-Processing

Before fitting the cubic polynomial to the waypoints, I transform them into the car reference system at time `t`. That is, a coordinates reference system with origin on the car poise, where `x=0` and `y=0` in the car center of gravity, and a yaw of 0 radians is aligned with the `x` axis. In this reference system, the car has of course coordinates `(0,0,0)`.

This allows to simplify the kinematic model, also for handling latency (se below), and allows me to send the optimisation results straight to the simulator, without further coordinates transformation. The simulator in fact expects a list of waypoints, and the predicted car positions at time `t, t+1, ..., t+N-1`, in the car reference system (at time `t`). 

## MPC with Latency

My program introduces a user-defined delay before sending the simulator the optimisation results. This is to simulate a latency time in actuators, i.e. a delay between the MPC directing the actuators, and the actuators applying any change.

An MPC, differently from a simple PID controller, is well equipped to handle latency, by incorporating it into the process model. My implementation predicts, at time `t`, what the car state will be at the end of the latency time; the predicted state vector is:

??? formula

Because the state is expressed in the car reference system at time `t`, the kinematic model gives the simplified state udpate equations:

The equations are further simplified by assuming that the car speed remains constant during the latency time, i.e. `a` is 0. This is of course an approximation, and the result is robust enough to to drive the car reliably around the track with a latency of up to 180 ms. Results vary depending on the performance and load of the computer running my program and the simulator, because they impact the controller response time, and potentially introduce further latency. 

## Parameters Tuning

Running the MPC implementation fist requires tuning: 
* `N`, the number of time-step in the finite-horizon optimisation;
* `Δt`, the time intervening between two consecutive time-steps, taken constant in this implementation;
 
Product `NΔt` gives the overall time, starting from `t`, during which the controller predicts and optimises the car trajectory. A too small `NΔt` doesn't allow the car to stay on track, as the controller doesn't look ahead enough; a too large one makes the computation heavier and doesn't bring actual benefits, as the model is approximate and its prediction accuracy degrades the more it goes into the future.
  
A small `Δt` would be desirable for more frequent, and therefore more accurate, optimisations. However, with a latency time of 100ms, I observed that setting `Δt` to 12 ms or lower makes the car behave erratically and go off-track, with a sweet spot around 13 ms.
  
I set `N` to 11 as the smallest value that gave me a smooth ride and a predicted trajectory subjectively close to the cubic interpolating the waypoints. A value smaller than 11 gives a predicted trajectory that often veers off visibly at its end, see picture below.

 

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
