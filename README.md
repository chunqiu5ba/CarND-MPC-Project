# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Rubric Points

### The Model
The vehicle model used in this project is a kinematic bicycle model. It neglects all dynamical effects such as inertia, friction and torque. The model takes changes of heading direction into account and is thus non-linear. The states for the model are [x,y,psi,v] which are position x and y, the yaw angle and the speed. The actuators are [delta,a] which are steering angle and acceleration. The model used consists of the following equations:
```
       x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
       y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
       psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
       v[t+1] = v[t] + a[t] * dt
       cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
       epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

```

### Timestep Length and Elapsed Duration (N and dt)
This project uses N=10 and dt= 0.1, it is considering a one second duration in which to determine a corrective trajectory. Larger N number will slow down the calculation and smaller number will not provide enough prediction ahead of the time. After testing N from 10 to 20 and dt from 0.1 to 0.5, numbers are finalized at 10 and 0.1

### Polynomial Fitting and MPC Preprocessing
The waypoints provided by the simulator are transformed to the car coordinate system 
```
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            ptsx[i] = x * cos(0-psi) - y * sin(0-psi);
            ptsy[i] = x * sin(0-psi) + y * cos(0-psi);
```
A third order polynomial is then fitted to the transformed waypoints. The polynomial coefficients are used to calculate the cte and epsi and these are used by the solver to create a reference trajectory.

### Model Predictive Control with Lantency
The state values are calculated by delay interval 100ms as there are 100ms delay in the code to simulate the latency of the actuations, implemented as:
```
          //State after delay
          double x_d = x0 + (v * cos(psi0) * delay);
          double y_d = y0 + (v * sin(psi0) * delay);
          double psi_d = psi0 - ( v * delta * delay / Lf);
          double v_d = v + a * delay;
          double cte_d = cte0 + (v * sin(epsi0) * delay);
          double epsi_d = epsi0 - (v * atan(coeffs[1]) * delay / Lf);
```
Cost function parameters were set after the vehicle can run smoothly on the track around 40mph, 
* 1000 delta
* 50 a
* 20000 delta(t+1) - delta(t)
* 300 a(t+1) - a(t)



## Dependencies

* cmake >= 3.5
* make >= 4.1(mac, linux), 3.81(Windows)
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

