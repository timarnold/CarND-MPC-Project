# Model Predictive Control

## Model

Model Predictive Control (MPC) is a control planning method that treats the
problem of following an optimal trajectory as an optimization problem. Given
a vehicle motion model and actuator values, the MPC algorithm predicts a
trajectory that the vehicle will follow. Optimizing over many such
trajectories for different parameter values, and minimizing a cost function
associated with each trajectory, the algorithm discovers the optimal actuator
values for thevehicle to execute the optimal path.

The equations of motion are just simple physics mechanics equations,
described in our Udacity lectures. Our state vector is composed of x, y, psi,
and v: position in two-dimensional space, an orientation angle, and a
velocity. Our cost function is the cross-track error (distance from intended
trajectory) and the error of predicted psi compared to an optimal psi value.
Our actuator values are the steering angle of the vehicle and the throttle
value.

The equations of motion, and cost functions, are as follows:

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

After finding the optimal trajectory, we return the steering angle and
throttle values that the vehicle uses to go forward one step. At the next
step, a new optimal trajectory is found using the same process described
above.

## Timestep Length, Elapsed Duration: `N * dt`

The time period for which MPC generates an optimal trajectory is defined by
the product of two variables, `N` and `dt`: the number of time steps and time
duration per step of the optimized trajectory.

In choosing these values, we trade off performance versus achieving enough of a
prediction into the future to have a reasonable chance of approximating the
future trajectory of the vehicle.

This time horizon should be on order ~a few seconds --- our current value is
~1 second. In addition to performance considerations, we need not calculate
too far into the future because an estimated trajectory far into the futureis
more likely to be incorrect by the time we arrive at that location anyway.

We tried other values for this, including ~20 or so for N and smaller values
for dt, but this combination of values seemed to produce ideal driving
behavior.

## Polynomial Fitting, MPC Preprocessing

The values of the intended trajectory are provided at each timestep by a
series of waypoints in the map's coordinate system. To use these waypoints to
fit an optimal trajectory, we transform the waypoints from map coordinate
space to the car's coordinate space, using a translation and rotation (the
car's coordinate space is translated by its position in map coordinate space,
and rotated by the heading value psi compared to the car's coordinate space).

After shifting coordinate spaces, we fit a 3rd-order polynomial to the
supplied waypoints to generate a desired trajectory. It is this 3rd-order
polynomial that we provide to our MPC solver, which then optimizes the
vehicle's predicted trajectory over all of our constraints (current state,
actuator values, etc.) to find the trajectory that most closely matches the
supplied optimal trajectory. Once this optimal trajectory is found, we
extract the next-step steering angle and throttle actuator values for that
trajectory and use them to adjust the car's actuators.

## MPC with Latency

To acommodate the latency of 100ms, we adjust all our state parameters so
they are advanced by the appropriate amount given the magnitude of the
latency, i.e., the state parameters are advanced so that they are at the
value they would beafter the latency period. The latency is currently 100ms.