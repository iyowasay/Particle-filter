# Localization project - Particle filter

The main goal of this project is to estimate the car's position using 2 dimensional particle filter in C++. Here the program communicates with Udacity Term 2 simulator via uWebSocketIO protocol. Particle filter is a continuous and dynamic state estimation which is commonly used in localization. Unlike Kalman filter, it can deal with non linear function and don't necessarily need to follow the assumption of Gaussian distribution. The importance weight shows how close(probability) a particle is to the actual position of the target. 

## Project Introduction

The vehicle has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. This is also known as Monte Carlo localization.

## Pipeline of particle filter

<img src="/image/steps.png" alt="steps" title="steps" width="500" height="250" />

1. Decide a suitable number of particles, time step, and define the noise from sensor datasheets(standard deviation or variance). 

2. Initialize the position of the vehicle with GPS information. Here three components(x,y coordinates and headings) are sampled from normal distribution. Though the accuracy of GPS reading is around 10 meters and usually not precise enough for localization of self driving cars, it can serve as a fast starting point of the filter. 

3. Make prediction based on previous believe, the motion model(bycycle model) and control data.

4. Receive sensor data from the simulator and transform into useful observation format. Since the car coordinate system is defined differently from the map coordinate system, we need to apply homogeneous transformation matrix to explicitly convert the coordinates.

5. Associate landmark observations with the closest landmark on the map(Nearest neighbor method). For instance, we obtain a lidar reading and need to associate it with an actual landmark in the real world. 

6. Apply the multivariate Gaussian probability density function, update the weights of each particle and calculate the posterior probability. Here we assume that each measurements and observations are independent, meaning that the final weight of a single particle can be determined by the product of probability density. 

7. Resample a new set of particles from current particles based on their weights. Particles with larger weights have higher probability of being picked. 

8. Repeat step 3. to 7. to acquire high density of particles and accurate localization.  

<img src="/image/result2.png" alt="success" title="success" width="500" height="250" />

## Reflection and discussion 

1. Data association 

Currently nearest neighbor method is implemented in the code. It's easy to understand and put into practice. However there are some disadvantages. For example, it does not take the sensor noise into account. Some errouneous detections might be included. It can also be influenced by the noise from the prediction step. Moreover, it is very inefficient when there are a lot of map landmarks and sensor measurements. More robust approaches can be considered to improve the accuracy. 


2. Sample and Resample 

- Rejection sampling(extremely slow)

- Importance sampling principle

Use a distribution function g(x)(e.g. Gaussian distribution) that you already know how to sample from it. Sample from this function. Obviously the sampling won't be perfect and there will be a difference between the target function`f(x)` and the selected function. Account for this difference by assigning a importance weight `w(x) = f(x)/ g(x)` to each sample. This is the underlying concept of particle filter.

- Resampling wheel

The objective of resampling is to generate a new set of particles with probability proportional to their weights. Since we can only have finite number of particles, a method called resampling wheel(linear time complexity) is implemented in the code. It starts with initializing two variables, BETA and index of weights. If the value of BETA is smaller than the weight at that index, we select the particle at that index. Otherwise we keep increasing the index by 1 and decreasing BETA until the condition is satisfied or we obtain enough number of particles. 


3. Error evaluation

Given the ground truth position of the moving vehicle, we are able to calculate the root squared error for evalutation. Usually the difference between ground truth and estimated position should be around 10 centimeters to be categorize as a good localization algorithm.

<img src="/image/eq1.png" alt="error" title="error" width="350" height="120" />


## Running the Code

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.


