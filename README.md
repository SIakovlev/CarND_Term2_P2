[//]: # (Image References)

# Unscented Kalman Filter Project

In this project I have utilized an Unscented Kalman Filter algorithm to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project [rubric](https://review.udacity.com/#!/rubrics/783/view). 

---

## Important Dependencies

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

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./UnscentedKF `

# Results

## Visualisation

The following graph compares real and estimated values for car coordinates using data from Dataset 1



## RMSE

The accuracy requirement is that the algortihm should perform with RMSE error lower than some threshold values. This shown in tables below for both datasets:

Dataset 1:

| Parameter | RMSE | RMSE threshold |
|:---------:|:----:|:--------------:|
|x          |0.0736| 0.10           |
|y          |0.0873| 0.10           |
|Vx         |0.3681| 0.40           |
|Vy         |0.2688| 0.30           |

Dataset 2:

| Parameter | RMSE | RMSE threshold |
|:---------:|:----:|:--------------:|
|x          |0.0| -           |
|y          |0.0| -           |
|Vx         |0.0| -           |
|Vy         |0.0| -           |

### EKF and UKF comparison

The following table compares RMSE values for [EKF](https://github.com/SIakovlev/CarND_Term2_P1) and UKF filters using Dataset 1:

| Parameter | EKF-RMSE | UKF-RMSE |
|:---------:|:----:|:--------------:|
|x          |0.0974  | *0.0736*         |
|y          |*0.0855*| 0.0873         |
|Vx         |0.4517  | *0.3681*         |
|Vy         |0.4404  | *0.2688*         |

### UKF: radar and/or lidar measurements:

| Parameter | UKF (L+R) | UKF (L) | UKF (R) |
|:---------:|:---------:|:-------:|:-------:|
|x          |*0.0736*   | 0.1148  | 0.1851  |
|y          |*0.0873*   | 0.1053  | 0.2774  |
|Vx         |*0.3681*   | 0.6386  | 0.3474  |
|Vy         |*0.2688*   | 0.3245  | 0.4307  |



