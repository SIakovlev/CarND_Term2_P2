[//]: # (Image References)
[image1]: ./pics/NIS_Radar.png
[image2]: ./pics/NIS_Lidar.png
[image3]: ./pics/Tracking.png

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
4. Run it: `./UnscentedKF `

# Results

## Visualisation

The following graph compares real and estimated values for car coordinates using data from Dataset 1

![alt_text][image3]

### Noise parameters
Part of the project was to adjust standard deviations `sta_a_` and `std_yawdd` so that RMSE lies below required thresholds. I have conducted experiments for different values of both parameters. The results are presented in the table below (to be short I used the following format: RMSE(`std_a_`, `std_yawdd_`)):

| Parameter | RMSE (3, 2) | RMSE (2, 2) | RMSE (2, 1) | RMSE (1, 1) | RMSE (0.5, 0.5) | RMSE (0.1, 0.1) |
|:---------:|:-----------:|:-----------:|:-----------:|:-----------:|:---------------:|:---------------:|
|x          |0.0736       | 0.0702      | 0.0701      | 0.0647      | **0.0612**          | 0.1250          |
|y          |0.0873       | 0.0858      | 0.0839      | 0.0837      | **0.0860**          | 0.1351          |
|Vx         |0.3681       | 0.3561      | 0.3446      | 0.3353      | **0.3304**          | 0.4175          |
|Vy         |0.2688       | 0.2541      | 0.2293      | 0.2195      | **0.2135**          | 0.3156          |

We can see how reducing the process noise parameters up to (0.1, 0.1) leads to worse RMSE. I've also plotted NIS (Normalised Innovation Squared) to perform consistency check for both radar and lidar sensors:

![alt_text][image1]

![alt_text][image2]

## RMSE

The accuracy requirement is that the algortihm should perform with RMSE error lower than some threshold values. This shown in tables below for both datasets:

Dataset 1:

| Parameter | RMSE (0.5, 0.5) | RMSE threshold |
|:---------:|:----:|:--------------:|
|x          |0.0612| 0.10           |
|y          |0.0860| 0.10           |
|Vx         |0.3304| 0.40           |
|Vy         |0.2135| 0.30           |

Dataset 2:

| Parameter | RMSE (0.5, 0.5) | RMSE threshold |
|:---------:|:----:|:--------------:|
|x          |0.0887| -              |
|y          |0.0611| -              |
|Vx         |0.6589| -              |
|Vy         |0.2822| -              |


### EKF and UKF comparison

The following table compares RMSE values for [EKF](https://github.com/SIakovlev/CarND_Term2_P1) and UKF filters (`std_a_, std_yawdd = (3, 2)`) using Dataset 1:

| Parameter | EKF-RMSE (3, 2) | UKF-RMSE (3, 2) |
|:---------:|:----:|:--------------:|
|x          |0.0974  | **0.0612**         |
|y          |0.0855  | **0.0860**         |
|Vx         |0.4517  | **0.3304**         |
|Vy         |0.4404  | **0.2135**         |

### UKF: radar and/or lidar measurements

The results are presented for `std_a_, std_yawdd = (3, 2)`:

| Parameter | UKF (L+R) | UKF (L) | UKF (R) |
|:---------:|:---------:|:-------:|:-------:|
|x          |**0.0736**   | 0.1148  | 0.1851  |
|y          |**0.0873**   | 0.1053  | 0.2774  |
|Vx         |**0.3681**   | 0.6386  | 0.3474  |
|Vy         |**0.2688**   | 0.3245  | 0.4307  |

## Rubric questions

### Compiling
Requirement is met. See above

### Accuracy
Requirement is met. See tables above

### Follows the Correct Algorithm

#### Processing flow
General processing flow is shown below: 
* Prediction step. Method `void UKF::Prediction(double)`.
  * Generate sigma points. Method `MatrixXd UKF::AugmentedSigmaPoints()`. 
  * Predict sigma points. Method `void UKF::SigmaPointPrediction(MatrixXd, double)`.
  * Predict mean and covariance. Method `void UKF::PredictMeanCovariance()`.
* Update step. 
  * Update step for Radar
    * Predict measurement. Method `void UKF::PredictRadarMeasurement(VectorXd*, MatrixXd*)`.
    * Update state. Method `void UpdateRadarState(VectorXd&, VectorXd&, MatrixXd&)`.
  * Update step for LIDAR
    * Predict measurement. Method `void UKF::PredictLidarMeasurement(VectorXd*, MatrixXd*)`.
    * Update state. Method `void UpdateLidarState(VectorXd&, VectorXd&, MatrixXd&)`. 

#### First measurement
After getting the first measurement UKF does the following:

* Initialisation of the state `x_` (depends on sensor) and state covariance matrix `P_`
* Update the time variable `time_us` and initialisation flag `is_initialised`
* Return to the `main()` function 

For implementation details see `void UKF::ProcessMeasurement(MeasurementPackage meas_package)`.

#### First predict then update
Requirement is met. For details see `void UKF::ProcessMeasurement(MeasurementPackage meas_package)`.

#### Radar and/or lidar measurements handling
This is done using flags `use_radar_` and `use_lidar` in the following lines from `void UKF::ProcessMeasurement(MeasurementPackage meas_package)`:

```c++
  Prediction(dt);
  if (meas_package.sensor_type_ == meas_package.RADAR && use_radar_)
    UpdateRadar(meas_package.raw_measurements_);
  else if (meas_package.sensor_type_ == meas_package.LASER && use_laser_)
    UpdateLidar(meas_package.raw_measurements_);
  else
    std::cout << "Skip measurement." << endl;
```
(the same logic is implemented for initialisation step)

### Code efficiency
See `\src\ukf.cpp` for source code.
