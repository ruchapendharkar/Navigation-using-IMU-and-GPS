# IMU and GPS Data Processing and Analysis

This project involved performing data processing and analysis on Inertial Measurement Unit (IMU) and GPS data collected during driving. The script includes three main parts:

## Part 1: Magnetometer Calibration and Yaw Analysis

### Magnetometer Calibration
- The raw magnetometer data is calibrated for hard and soft iron effects.
- The calibrated magnetometer data is plotted before and after calibration.

### Yaw Analysis
- Yaw is computed from IMU quaternion data and magnetometer data.
- Comparison of yaw from gyroscope and magnetometer is performed.
- Sensor fusion using a complementary filter is applied to combine gyroscope and magnetometer estimates.

## Part 2: GPS Data Processing

### GPS Velocity Estimation
- Velocity is estimated from GPS data.
- Outliers in GPS velocity are corrected.

### IMU Velocity Estimation and Correction
- Velocity is estimated from accelerometer data.
- Outlier detection and correction are applied to improve IMU velocity estimation.

## Part 3: Position Estimation

### Displacement Estimation
- Displacement is calculated from both GPS and IMU velocity.
- UTM coordinates are used to plot the route on a graph.

### Additional Analysis
- Comparison of observed acceleration and Coriolis acceleration.

## Usage
- The script is designed to be run in MATLAB.
- Ensure required CSV files ("imu.csv", "driving_imu.csv", "driving_gps.csv") are present in the same directory.

## ROS Integration
- To integrate with ROS, use the following command to run the launch file:

    ```
    $ roslaunch LAB4 driver.launch port1:="(name of IMU port)" port2:="(name of the GPS port)"
    ```

- Make sure to replace "(name of IMU port)" and "(name of GPS port)" with the actual port names.
- The name of the ROS package is LAB4.

## Dependencies
- MATLAB (R2022a or later)
- ROS

