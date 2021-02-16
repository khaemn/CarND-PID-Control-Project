# CarND-Controls-PID Writeup
Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


## Overview

The project implements two PID controllers: for a vehicle speed and for a vehicle steering angle. An example video of the car, driving in the [Term2 simulator](https://github.com/udacity/self-driving-car-sim/releases), can be found [here on YouTube](https://www.youtube.com/watch?v=0J0UKa9l3Ok)


### PID formula

I use the discrete PID controller formula, below is a pseudocode, roughly describing the algorithm I have been using to compute the controlling value (e.g. throttle or steering angle):

```
const T = 0.02 # discretisation period in seconds, 20 ms in the available simulator

integral_err_sum = 0
prev_err = 0

for ... :
    err = desired_value - current_value

    integral_err_sum += err * T

    U = (-1) * (Kp * err   +   Ki * integral_err_sum   +   Kd * (err - prev_err) / T)
    
    prev_err = err
```

Where U is a concrete value of the controlling parameter (e.g. steering angle or a throttle). Of course, after the calculation the controlling parameter is also being clamped to an allowed range (-1..1 for steering, 0..1 for throttle).


### PID coefficients tuning algorithm

In order to use the twiddle algorithm, described in lectures, or a gradient descent method, or some other analythical methods, it is necessary to have a stable and reproducible experimental environment.
Alas, even in the simulator (which is by far simpler than the real world conditions) it is not quite possible, as the car is being controlled by the two PID regulators (speed and steering) simultaneously, and there is no convenient way to run hundreds of steps for twiddle or gradient descent, maintaining 100% reproducible conditions.

So I have used a simplified algorithm of PID coefficients tuning, described in many sources across the Internet, (here)[https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops] for example. I have used this algorithm before in some job tasks, for example to control an air flow via a flap, powered by a DC motor. This algorithm requires the system to allow oscillations and allow over-reacting without destruction, and also it requires that the controlled process is "fast" enough to observe the oscillations, but not too "fast" so that the oscillations lead to destruction or are too hard to observe. THe car steering angle is a perfect target for such an algorithm, as the oscillations around lane center is easily observable, yet in the simulator the car is safe to be crashed due to too large oscillations. The same for its speed: the oscillations are clearly visible, and it is safe (in the simulator) to go overboard with the speed.

In a nutshell, the steps are:

- set all 3 coeffs to 0
- set Kp to some start value (I have used 0.1) and observe the system behavior:
  * while the system does not change its behavior or a change is negligible: increase Kp 10 times
  * while the system starts oscillating with increasing magnitude: decrease Kp 10 times
  * when there is a Kp range, in which the system oscillates but keeps around a desired value, keep increasing or decreasing the Kp with small deltas until there's a minimum magnitude of oscillations found.
- set Kd to some initial value (I have used 0.01) and observe the system behavior:
  * while the system does not change its behavior (oscillates as before) or a change is negligible: increase Kd 10 times
  * while the system's oscillations decrease: keep increasing Kd with some small delta
  * if the oscillations are growing: decrease Kd
  * do above until there is such a Kd found, that the system only does a couple of oscillations after a rapid change in the controlled value, but with a stable controlled value there is no oscillations or they are negligible.
- set Ki to some _small_ initial value (I have used 0.001) and:
  * Increase Ki 10 times or with a smaller multiplier, or a small delta, until the system would keep the desired value without gaps
  * If the system starts oscillating - decrease Ki

Performing of these steps in real life demands writing down all the coefficient values and system reaction on each step, and can be time-consuming, but it guarantees finding some working set of coefficients (if it is possibe at all).


### PID coefficients tuning issues

I was not able to found any set of Kp, Ki, Kd, that would keep the car on track at _any_ speed from 30 to 75 MPH. It turned out, that the coefficients, that "drives" the car through the whole track at 55 KMH (0.052, 0.03, 0.0135 respectively) do not work decently at 25..30 MPH, and, likewise, a coefficient set for the steering PID, that works fine at 30 MPH (0.04, 0.002, 0.02) leads to almost an immediate crash at 55..60 MPH.
Also there are 2 simultaneous oscillations - of the car's speed and its direction - so when they sometimes going to a resonance, it always ends up badly. I have dealed with it making the speed regulator as smooth as possible (though it leads to a bit greater errors on high speeds, for examples, at 60 MPH installed it can drive about 62 MPH).


### Conclusions

I have tested about 30 sets of the PID coefficients for speed controller, and over 200-250 sets of the PID coefficients for the steering controller, using the tuning algorithm described above, and was able to tune a parameter set, that drives ok in range 25..45 MPH. Unfortunately, I did not manage to tune the regulators enough to drive faster than 70 MPH (and even at that point the car was already behaving unsafely at at the steep curves), so, indeed, the tuning of a PID regulator is not a simple task and requires a lot of time and efforts to be done properly, especially for dynamic and unstable systems such as a moving car.
It turned out, that it is more important to tune the regulator for the highest possible speed, as for the lower speeds it would behave not too worse; but it is not true vice versa: a regulator, that drives perfectly on a low speed, could cause crash even at a speed larger only by 10..15% than the tuning speed.
In the `main.cpp` in the `steer_pid.Init( ... )` and the `throttle_pid.Init( ... )` calls I list the coefficients for both regulators which I consider the best so far.
The speed (55MPH), set in the `main.cpp`, is the highest at which the car still behaves safely (though touching yellow and/or red lines on the track eventually); the coefficients would work up to 65 MPH as well, but the car's behavior becomes much more agressive and I don't consider this to be allowed.
