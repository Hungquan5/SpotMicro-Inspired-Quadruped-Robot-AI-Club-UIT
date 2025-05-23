# SpotMicro-Inspired Quadruped Robot AI Club UIT

## Overview

SpotMicroBot is a quadruped robot inspired by the open-source SpotMicroAI project, designed for cost-efficient, high-performance robotics. Our project enhances the original design with custom 3D-printed components, precise control using Jetson Nano with ROS, and advanced capabilities in computer vision and autonomous navigation.

## Inspiration

This project is inspired by SpotMicroAI, a popular open-source quadruped robot platform. For detailed information about SpotMicroAI, please visit the official documentation: [SpotMicroAI Documentation](https://spotmicroai.readthedocs.io/en/latest/) and the GitHub repository: [SpotMicroAI GitHub Repository](https://github.com/mike4192/spotMicro).

## Key Features

* Custom 3D-printed parts for cost-efficient, high-quality construction.
* Jetson Nano with ROS for precise control and smooth movement.
* Camera and LiDAR integration for advanced computer vision tasks.
* Autonomous navigation using reinforcement learning (RL).
* Virtual environment testing using Genesis for efficient training.

## 3D Printing

We utilized an in-house custom-built 3D printer for manufacturing high-quality parts, ensuring precise design control and cost-effectiveness. Our setup was carefully calibrated to achieve dimensional accuracy and durability, resulting in a reliable, robust robot.

## Control System

Powered by Jetson Nano with ROS (Robot Operating System), SpotMicroBot offers precise control with minimal delay, ideal for complex tasks like reinforcement learning-based autonomous navigation. The system demonstrates seamless hardware-software integration.

## Inverse Kinematics

We implemented inverse kinematics (IK) for precise leg control, calculating joint angles for smooth and stable movement. Genesis was used as a virtual environment for safe, efficient IK testing and optimization. This IK implementation references:

* Sen, Muhammed Arif & Bakircioglu, Veli & Kalyoncu, Mete. (2017). Inverse Kinematic Analysis Of A Quadruped Robot. *International Journal of Scientific & Technology Research. 6.*

## Virtual Environment - Genesis

Genesis was used as a virtual environment for training and testing, providing a safe and efficient platform for optimizing the robot's navigation algorithms and refining its control strategies.

## Future Development

### Enhanced Autonomous Navigation

* Implementing LiDAR and camera integration for improved perception and obstacle detection.
* Developing advanced reinforcement learning (RL) algorithms for efficient and adaptive autonomous navigation.
* Using the Genesis virtual environment to train and test the navigation model, ensuring safe and efficient learning.

### Computer Vision Capabilities

* Utilizing the camera for computer vision (CV) tasks, including object recognition, obstacle avoidance, and terrain analysis.
* Integrating deep learning models for real-time CV processing on Jetson Nano, leveraging its GPU acceleration.

### System Optimization

* Enhancing the inverse kinematics (IK) system for more precise and adaptive leg control.
* Further calibrating the custom 3D-printed parts for improved structural integrity and performance.
* Streamlining the ROS architecture for better resource management and lower latency.

### User Interface and Control

* Developing a user-friendly control interface for manual and autonomous modes.
* Enabling remote monitoring and control through a web interface or mobile app.

These developments aim to make SpotMicroBot a versatile, intelligent, and adaptable quadruped robot suitable for a wide range of applications.


## Citation

* [SpotMicroAI Documentation](https://spotmicroai.readthedocs.io/en/latest/)
* [SpotMicroAI GitHub Repository](https://github.com/mike4192/spotMicro)
* Sen, Muhammed Arif & Bakircioglu, Veli & Kalyoncu, Mete. (2017). Inverse Kinematic Analysis Of A Quadruped Robot. *International Journal of Scientific & Technology Research. 6.*
