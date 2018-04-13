# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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
  * Run `install-ubuntu.sh`.

* **Ipopt and CppAD:** 

  * ` sudo apt-get install gfortran`
  *  `apt-get install unzip`
  *  `wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip`
 *  `sudo ./install_ipopt.sh ../Ipopt-3.12.7`

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo

* Simulator. Download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).


## Compiling and executing the project
1. Clone this [repo](https://github.com/udacity/CarND-MPC-Project)
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
5. Now the MPC controller is running and listening on port 4567 for messages from the simulator. 
6. Open Udacity's simulator and validate the connection established
7. Navigate to Project 5: MPC Controller
8. Click on the "Select" button

## Implementation
---
### The Model
The model used is a Kinematic model neglecting the complex interactions between the tires and the road. The model equations are as follow:
 
```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

Where:

  * x, y : Car's position.
  * psi : Car's heading direction.
  * v : Car's velocity.
  * cte : Cross-track error.
  * epsi : Orientation error.

Those values are considered the state of the model. In addition to that, Lf is the distance between the car of mass and the front wheels (this is provided by Udacity's seed project). The other two values are the model output:

* a : Car's acceleration (throttle).
* delta : Steering angle.

### Timestep Length and Elapsed Duration (N & dt)
The number of points(N) and the time interval(dt) define the prediction horizon. The number of points impacts the controller performance as well. I tried to keep the horizon around the same time the waypoints were on the simulator. With too many points the controller starts to run slower, and some times it went wild very easily. After trying with N from 10 to 20 and dt 100 to 500 milliseconds, I decided to leave them fixed to 10 and 100 milliseconds to have a better result tuning the other parameters.

### Polynomial Fitting and MPC Preprocessing
The waypoints provided by the simulator are transformed to the car coordinate system at ./src/main.cpp from line 104 to line 113. Then a 3rd-degree polynomial is fitted to the transformed waypoints. These polynomial coefficients are used to calculate the cte and epsi later on. They are used by the solver as well to create a reference trajectory.

### Model Predictive Control with Latency
To handle actuator latency, the state values are calculated using the model and the delay interval. These values are used instead of the initial one. The code implementing that could be found at ./src/main.cpp.

## Simulation
Working fine as per rubric points