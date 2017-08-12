# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Project

* Using Model Predictive Control (MPC), this project involves writing a C++ program that can drive a simulated car around a virtual track. The simulated model has a 150ms latency that is taken into account while processing.
 * Fit a line following waypoints provided to show vehicles current and future locations
 * Implement MPC calculations including setting variables and constraints
 * Account for latency (100ms was the default latency, I changed this to 150ms)
 * Adjust steering angle and throttle based on calculated actuator values
 * Set a timestep for calculating, and test on Udacity Term 2 Simulator
 
 ## The Model
 
 * My model implemented much of the quiz code as well as code from the video tutorial. First, the waypoints needed to be retrieved and stored from the initial car position. The initial x and y points were translated so that the front of the car would be (0,0). Any other points after this were also translated to keep this same orientation. All other initial values for the controller were set to 0 as well since the car was not yet moving. 
 * Now that we have all of the initial values, let's get this car moving. The original N (timestep length) and dt (elapsed duration between timesteps) values were sufficient to have the model work properly, but I wanted to test out different ones and see if I could still get a decent model. I tested dt at .15, keeping N at 10 and this still kept a sufficient model. There was a time when I put N up to 15 and dt at .2 just to see what would happen, and the car went crazy and was off the road within a few seconds. Because of this, I decided to stick with values I had working and continue on. I also changed the latency of the vehicle (in main.cpp) to be .15 so the model would predict where the car was going to be in the next 1.5 seconds. 
 * Some predictions of waypoints are made before being solved for:
  * psi was predicted based on the velocity, current steering angle, and the car's center of gravity (this would initially be 0 based on the equation) ```pred_psi = 0.0 + v * -delta / Lf * dt```;
  * velocity was determined based on current velocity and how much throttle was being used at that time (initially 0 based on the equation) ```pred_v = v + a * dt```
  * cte was predicted based on the current cte as well as velocity, error in psi, and the latency; ```pred_cte = cte + v * sin(epsi) * dt```
  * epsi was predicted using the current epsi with velocity, steering angle, the car's center of gravity, and the latency (this would also initially be 0 due to starting with epsi = 0 and the velocity and steering angles equal to 0) ```pred_epsi = epsi + v * -delta / Lf * dt```
 * The equations that I used were all equations given from the video tutorial, however, I did change some of the weights. I noticed that with the current weight on the steering angle, the car got very close to going off the road on the curve coming off the bridge. I made that a little bit higher so turns could be a little bit sharper. I notice that because of this the speed of the vehicle slows down more, but the vechile staying on the road is more important than how fast it gets around the track. 
 * The final points are then put into polyeval() which ends up creating a best fit 3rd degree polynomial for the waypoints, and then are plotted to the track. The given waypoints are drawn in yellow while the ones for the car's path are drawn in green. A final video of a lap around the track can be seen [here](CarND-MPC-Project-Lap.mov)
 
 
---

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
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

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
