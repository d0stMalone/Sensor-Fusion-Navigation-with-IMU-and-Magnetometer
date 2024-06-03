IMU and Magnetometer-Based Navigation

Overview
This repository contains the code and data analysis for the project "Navigation with IMU and Magnetometer." The project aims to estimate heading and velocity using data from an IMU and a magnetometer, and to perform dead reckoning for navigation.

Analysis
Heading Estimation

Magnetometer Calibration: Corrected for hard iron and soft iron distortions.
Complementary Filter: Combined yaw estimates from the magnetometer and gyroscope.

Forward Velocity Estimation
GPS Data: Used to estimate velocity and align with IMU data.
Accelerometer Data: Integrated to estimate velocity, with corrections for biases.

Dead Reckoning
Displacement Estimation: Integrated GPS and IMU velocities to estimate displacement.
Trajectory Analysis: Compared IMU trajectory with GPS trajectory.

Results and Discussion
Noise Characteristics: Magnetometer data corrected for distortions, but IMU data showed biases.
Performance Comparison: GPS data provided more accurate velocity estimates compared to IMU data.
Environmental Noise Sources: Factors like mechanical, electrical, and environmental noises affected measurements.

Conclusion
The project successfully estimated heading and velocity using IMU and magnetometer data. Dead reckoning provided insights into sensor performance and highlighted the importance of calibration and noise reduction.

References
Complete Video of Data Collection
VN100 Datasheet