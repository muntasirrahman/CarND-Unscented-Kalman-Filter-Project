# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

---
[//]: # (Image References)
[img_result]: images/result.jpg "Result"

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.


## Obtained RMSE Value

![Result][img_result]


# [Rubric points](https://review.udacity.com/#!/rubrics/783/view)

## Compiling

### Your code should compile.

There is no change on CMakeList.txt.

## Accuracy

### For the new version of the project, there is now only one data set "obj_pose-laser-radar-synthetic-input.txt". px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30] when using the file: "obj_pose-laser-radar-synthetic-input.txt"

The obtained RMSE value are:

| RMSE | Value  | Passing Criteria |
|:----:|:------:|:------:| 
| px   | 0.0693 | 0.09 |
| py   | 0.0835 | 0.10 |
| vx   | 0.3336 | 0.40 |
| vy   | 0.2380 | 0.30 |

## Following the Correct Algorithm

### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

The general processing are file in [src/ukf.cpp](./src/ukf.cpp):
 * method ProcessMeasurement at line 96
 * followed by method Prediction at Line 155,
 * followed by method UpdateRadar (Line 201) or method UpdateLidar (Line 273) depending on the measurement data type

### Your Kalman Filter algorithm handles the first measurements appropriately.

The first measurement is handled at [src/ukf.cpp](src/ukf.cpp) method ProcessMeasurement from line 104 to line 131.

### Your Kalman Filter algorithm first predicts then updates.
This is part method ProcessMeasurement update from line 137 to 145.

### Your Kalman Filter can handle radar and lidar measurements.

Radar measurement data is handled at method UpdateRadar of [src/ukf.cpp](src/ukf.cpp) from line 201 to line 267.

Lidar measurement data is handled at method UpdateLidar of [src/ukf.cpp](src/ukf.cpp) from line 273 to line 365.



