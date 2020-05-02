# **Kidnapped Vehicle Project**


**The goals / steps of this project are the following:**
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). See README_orig.md for getting the environment up and running.

[//]: # (Image References)

[image1]: ./writeup_images/completed.png "Successful run "
[image2]: ./writeup_images/overview.png "Overview of filter process"
[image3]: ./writeup_images/filter_comparison.png "Comparison of different filter techniques"

---

## Writeup / README

A Particle filter is an easy to program, but very effective way of estimating a vehicle's location based on imperfect sensor data and imperfect vehicle movement information. Other solutions also exist for this same problem, with different advantages and disadvantages, see this comparison by Sebastian Thrun from the course material:

![image3]


The basic idea is that the program creates a large number of virtual cars, the particles, and moves each particle based on the fused sensor information received from the car, plus an added noise to counteract measurement errors. Then, after each sensor measurement, the program goes through each particle and compares what the particle should be seeing based on the map objects and the particle position/heading with what the sensors are seeing. Particles that should see something very similar to what the real-life sensors are seeing get higher weights, and the ones that should see something very different get lower weights. After each measurement the particle set is randomly resampled, which means that particles with higher weights tend to multiply, while ones with lower weights tend to die out. 

See this overview flow chart from Tiffany Huang from the course material:
![image2]

The `src/particle_filter.cpp` source file implements these steps. 

The `::init` function creates `num_particles` new particles, scattered randomly around an initial position estimate (the less accurate the position is, the more and more scattered these particles should be). I configured the number to 20 particles, as, under the simulation scenario, as this number is already high enough to produce fairly good results, but low enough not to require excess computation.

The `::prediction` function moves each particle along based on their current heading as well as the current real-world yaw-rate and velocity measurement (plus noise to counteract sensor error). It it using the motion model that considers yaw rate when yaw rate is non-zero, and a simplified motion model when yaw rate is 0.

The `::updateWeights` function updates the weight of each particle based on how closely the predicted measurements (based on the particles position and heading as well as the map data) match the actual sensor measurements (possibly a fused measurement from lidar, radar and other sensors). For each particle the function performs the following steps: 
* translation of measurements from the vehicle's coordinate system to the map coordinate system based on particle position and heading.
* creation of predicted landmark measurement list, based on the map, particle position and heading
* matching the predicted measurement list with the sensor measurement list, based on a nearest neighbor approach
* setting a new weight for the particle applying the Gaussian Probability Density function to each predicted - observed measurement and multiplying the results

Finally, the `::resample` function performs the resampling. It iterates `num_particles` of times, and each time it randomly selects a current particle with a probability proportional to the particle weights. These particles are added to a new list, and finally the old list is replaced with the new list.

        
## Testing

The code was compiled in WSL Ubuntu on Windows and tested against the Windows build of the Term2 simulator. It maintained a typical error of 0.1 m in each direction (the target was to stay under 1m) and ran in 48 seconds on my PC (the target was less than 100s -- but the same code ran for around 100 seconds on my laptop, so the performance evaluation criterion is quite inexact).
 
 Here is the simulation result:
![image1]
