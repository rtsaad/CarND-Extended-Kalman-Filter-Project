# Extended Kalman Filter Project

This project consists of a c++ implementation of an Extended Kalman Filter (EKF) to estimate the state (position and velocity) of a moving object using noisy Lidar and Radar sensors. This project improves the estimations by using the Kalman Filter to fuse the sensor readings. 

The main goals of this project is to develop a c++ kalman filter that successfully estimates the position of a moving object from the Udacity Simulator. Figure 1 depicts an example of the filter estimating (green dots) the object position. The RMSE (Root Mean Square Error) values estimates the accuracy of the EKF.

![alt text][image1]

[//]: # (Image References)

[image1]: images/fusion.png "Fusion Sensor Estimation"
[image2]: images/radar.png "Radar Sensor Estimation"
[image3]: images/lidar.png "Lidar Sensor Estimation"

## 1.Access 

The source code for this project is available at [project code](https://github.com/otomata/CarND-Extended-Kalman-Filter-Project).

## 2.Files

The following files are part of this project:
* src/kalman_filter.*:     Kalman Filter class definition;
* FusionEKF.py:   Extended Kalman Filter class definition;
* tools.py:       helper functions to compute rmse errors and jacobian matrix for linear approximation;
* images: 
** fusion.png:  Estimation using Lidar and Radar readings;
** lidar.png:    Estimation using only Lidar readings;
** radar.png: Estimation using only Radar readings;

### Dependency

This project requires the following python packages to work:
* Udacity Simulator [https://github.com/udacity/self-driving-car-sim/releases/](https://github.com/udacity/self-driving-car-sim/releases/);
* cmake 3.5 or above;
* make 4.1 or above;
* gcc/g++: 5.4 or above;
* uWebSocketIO;

### WebSocketIO

This project uses the open source package called WebScokectIO to facilitate the communication between the EKF and the Udacity Simulator. To install all the websocketio libs, execute the script ``install-ubuntu.sh`` from the project repository directory.

## 3.How to use this project

To run this project, you first need to compile the code. After the code is compiling, please, open the Udacity simulator than 

### Compiling and Running

The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF
6. Run the Udacity Simulator (./term2_simulator)

## 4.Results

Figure 2 shows the EKF estimation using only RADAR readings. The RMSE values of 0.23, 0.33, 0.60, 0.68 show the EKF accuracy to estimate the moving object position and velocity. 

![alt text][image2]


Figure 3 shows the EKF estimation using only LIDAR readings. As we expected from the LIDAR and RADAR covariance matrix, the accuracy is higher using only the LIDAR sensor. The RMSE values of 0.18, 0.15, 0.60, 0.48 show the EKF accuracy to estimate the moving object position and velocity. 

![alt text][image3]

Finally, Figure 4 shows the EKF estimation using RADAR and LIDAR readings. The RMSE values of 0.09, 0.08, 0.45, 0.43 show the high accuracy the Sensor Fusion promotes to estimate the moving object position and velocity.

![alt text][image1]


