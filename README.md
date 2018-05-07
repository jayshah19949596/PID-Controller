# CarND-Controls-PID
---
Self-Driving Car Engineer Nanodegree Program.
This repository contains C++ code for implementation of PID controller, to derive steering angles for a car to drive around a circular track. 
This task was implemented to partially fulfill Term-II goals of Udacity's self driving car Nano degree program.


## Background
---
A critical module in the working of any robotic system is the control module. Control module defines the action, which the robotic system performs in order to achieve a task. These actions can vary on type of the system and type of the task. For e.g.: A simple mixer grinder's control module only controls the speed of rotating motor. A little more complex system such as a Remote Controlled (RC) car needs a control module to move forward, backward and turn. Highly complex systems such as prosthetic arms, self driving cars, product manufacturing factory units require control modules for multiple tasks at the same time.

One of the basic implementation of a control system is a Proportional (P), Differential (D), Integral (I), together, a PID controller. PID controller is the most popular controller and is used in applications across domains. 

## PID-Controller
---
A proportional–integral–derivative controller (PID controller or three term controller) is a control loop feedback mechanism widely used in industrial control systems and a variety of other applications requiring continuously modulated control. 
A PID controller continuously calculates an error value `e(t)` as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively) which give the controller its name.

## Working of PID Controller
---
The basic principle of working of a PID controller is to satisfy a boundary value problem. Most common examples of such problems are either minimization of the total error in the system or maximization of the total gain in the system. These error or gain problems are represented mathematically and are used to govern the effect to P, I and D components of a PID controller. These components are described below:

  1. Proportional (P) component:
    Mathematically, the P component establishes linear relationship with the problem. The effect of P component is in proportional to the value of problem at the current time step. For e.g.:, if the problem is to minimize the error in the system at current time step, the value of P component is given by the formula:
    
    αp = -Kp * error
    
    where, Kp is a tuning parameter known as the proportional gain. The negative sign in the beginning signifies that P component is used to cancel the effect or error, or in other words, reduce it.
    
  2. Differential (D) component:
    Mathematically, the D component establishes linear relationship with the rate of change of problem. The effect of D component is in proportional to the rate of change problem from last time step to current time step. For e.g.:, if the problem is to minimize the uncertainty in the system, the value of D component is given by the formula:
    
    αd = -Kd * d(error)/dt
    
    where, Kd is a tuning parameter known as the differential gain. The negative sign in the beginning signifies that D component is used to cancel the effect or rate of change of error, or in other words, reduce it. The D component is used to minimize sudden changes in the system.
    
  3. Integral (D) component:
    Mathematically, the I component establishes linear relationship between the average value of problem over time. The effect of I component is in proportional to the average value of problem from the beginning of time to the current time step. For e.g., if the problem is to minimize the error in the system, the value of I component is given by the formula:
    
    αi = -Ki * ∑ error
    
    where, Ki is a tuning parameter known as the integral gain. The negative sign in the beginning signifies that I component is used to cancel the effect or average error, or in other words, reduce it. The I component is used to correct systemic bias.
    
  When all components are used, the mathematical equation is given by:
  
  α = (-Kp * error) + (-Kd * d(error)/dt) + (Ki * ∑ error)
  
  where, α is the control input to the system, often known as the **actuator** input.
  

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

