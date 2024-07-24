# Welcome to B.OB's Wiki
B.OB is a differential drive robot which is controlled using ROS2 Humble running on a Raspberry Pi 4. 

## Current status
B.OB currently has 2 driving modes, Basic and Assisted. 
* Basic Driving is teleoperation via a controller using no extra sensors.
* Assisted Driving uses sensors to enhance the capabilities of bob such as:
  * Mapping with SLAM using Lidar

    ![Bob_SLAM](https://github.com/user-attachments/assets/b936f409-9442-4920-a44e-085fce1e72bc)

  * Lateral controller using the IMU

    ![Bob_PID](https://github.com/user-attachments/assets/b326bcfc-d341-4084-8f6a-dea6a1f88bd6)

  * Localization by fusing the IMU sensor and wheel encoders via an Extended Kalman Filter(EKF).

    ![Bob_IMU](https://github.com/user-attachments/assets/c920546d-8ee9-42b5-9404-a9bc3db44e03)

## Next Goal
Autonomous driving is the next goal for B.ob.

![Autonomous_navigation](https://github.com/user-attachments/assets/7756cd2e-c514-40a8-bd17-661a63059be5)

Note: This video is done purely in simulation
