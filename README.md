# RALFAS-OCV: ROS2 Autonomous Lane Following Robot with Ackerman Steering using OpenCV

Welcome to the **RALFAS-OCV** project repository! This project was developed as a mid-term project for the course *EGR 530: Principles of System Engineering* at **Arizona State University** in the *Spring of 2024*. Under the guidance of [Professor Junfeng Zhao](https://www.linkedin.com/in/junfeng-zhao-33139572/), I have created an autonomous robot car that uses Ackerman steering and advanced computer vision techniques for precise lane following.

## Project Overview

This project aims to develop an autonomous robot with Ackerman steering and computer vision capabilities to follow lanes accurately. The system utilizes a Raspberry Pi to process visual input from a camera module, identify the path, and make real-time steering mechanism adjustments.

## Key Highlights

### Hardware Components
- **Raspberry Pi 4B**: Core processing unit.
- **L298N Motor Driver**: Motor control.
- **Two 12v to 5v Buck Convertors**: Power management.
- **Two 12V DC Motors (550 rpm)**: Movement.
- **7.4V 20KG Servo Motor**: Steering.
- **Raspberry Pi v1 Camera**: Captures video feed.
- **Miscellaneous**: Fuse, switch, jumper wires, terminal connectors.
- **Chassis**: Yahboom Rosmaster R2.

### Software Components
- **Operating Systems**: Ubuntu 22.04 on the main computer and Raspberry Pi.
- **Development Tools**: VS Code, ROS2 Humble.
- **Libraries**: OpenCV for image processing, CvBridge for image conversion between ROS and OpenCV.

### Algorithm
- **Real-time Image Processing**: Canny edge detection and Hough Line Transform to detect lane lines.
- **Midpoint Calculation**: Calculates the midpoint of lanes and the robot's lateral deviation.
- **Steering Adjustment**: Dynamic adjustments to steering based on real-time calculations.
- **Communication**: Efficient data transfer using Cyclone DDS.

## Project Achievements
- **Real-time Navigation**: Smooth operation with real-time visual feedback and dynamic adjustments.
- **Edge Detection**: Effective lane line identification and accurate error calculation for lateral deviation.
- **Modular Design**: Separate nodes for image acquisition, processing, and steering control, enhancing maintainability and scalability.

## Future Enhancements
- **Improved Camera Alignment**: For better lane detection.
- **Advanced Path Planning**: To handle smoother curve navigation.
- **Integration of Deep Learning**: For traffic sign recognition.
- **Obstacle Detection**: Using Lidar or depth cameras for 3D mapping and SLAM.

## Usage
- **Image Acquisition**: Captures and publishes real-time video feed from the Raspberry Pi camera.
- **Lane Detection**: Processes images to detect lane lines and calculates deviation.
- **Steering Control**: Adjusts steering based on calculated deviation to maintain lane position.

## License
- This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments
- Professor [Professor Junfeng Zhao](https://www.linkedin.com/in/junfeng-zhao-33139572/): Thank you for your invaluable guidance and support throughout the project.

## Demo



https://github.com/AryanKumarNadipally/RALFAS-OCV-ROS2-Autonomous-Lane-Following-Robot-with-Ackerman-Steering-using-OpenCV/assets/143588978/854bed32-3ce7-46be-a3c3-4b6354a7f341

