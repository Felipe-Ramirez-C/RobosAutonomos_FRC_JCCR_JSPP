<a id="readme-top"></a>

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]

<br /> <div align="center"> <h3 align="center">Autonomous Human Search and Positioning</h3>

<p align="center"> A ROS 2 Humble project for autonomous navigation, human detection using YOLOv8, and side-positioning with a Clearpath Husky A200. <br /> <a href="https://github.com/your_username/your_repo"><strong>Explore the docs »</strong></a> <br /> <br /> <a href="https://github.com/your_username/your_repo">View Demo</a> · <a href="https://github.com/your_username/your_repo/issues">Report Bug</a> · <a href="https://github.com/your_username/your_repo/issues">Request Feature</a> </p> </div>




<details> <summary>Table of Contents</summary> <ol> <li> <a href="#about-the-project">About The Project</a> <ul> <li><a href="#built-with">Built With</a></li> </ul> </li> <li> <a href="#getting-started">Getting Started</a> <ul> <li><a href="#prerequisites">Prerequisites</a></li> <li><a href="#installation">Installation</a></li> </ul> </li> <li><a href="#usage">Usage</a></li> <li><a href="#roadmap">Roadmap</a></li> <li><a href="#contact">Contact</a></li> <li><a href="#acknowledgments">Acknowledgments</a></li> </ol> </details>

<!-- ABOUT THE PROJECT -->
This project addresses the challenge of autonomous exploration and human interaction in a complex "office" environment. The robot, a Clearpath Husky A200, must autonomously navigate a partially unknown space to find a person, signal their location, and position itself precisely next to them.

Key features:
100% Autonomy: No teleoperation or external commands during the task.
Intelligent Perception: Utilizes YOLOv8 for human classification and pose identification.
Wall-Following Logic: Custom LiDAR-based navigation for environment exploration.
Automated Reporting: Generates a log file with the person's location and total mission time.
<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With
* [![ROS2][ros2-shield]][ros2-url]
* [![Python][python-shield]][python-url]
* [![Gazebo][gazebo-shield]][gazebo-url]
* [![YoloV8][yolov8-shield]][yolov8-url]
 
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

Prerequisites:
* Ubuntu 22.04 LTS 
* ROS 2 Humble Hawksbill 
* Gazebo Simulator
* Clearpath 
* YOLOv8 (Ultralytics)

### Installation
Install Clearpath Husky simulation packages
  ```sh
   git clone https://github.com/github_username/repo_name.git
   ```

Bash sudo apt install ros-humble-clearpath*
Create the required configuration folder and robot.yaml10:Bashmkdir -p ~/clearpath
cp path_to_your_config/robot.yaml ~/clearpath/robot.yaml
Clone the repository into your ROS 2 workspace:Bashcd ~/ros2_ws/src
git clone https://github.com/your_username/your_repo.git
Install Python dependencies for YOLOv8:Bashpip install ultralytics
Build the workspace:Bashcd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage
To launch the full simulation, including the environment, robot, and autonomous nodes11:Bashros2 launch your_package_name simulation.launch.py
Known Simulation FixesIf the robot model or TFs do not appear in RViz automatically, run the visualization launch in a separate terminal12121212:Bashros2 launch clearpath_viz view_robot.launch.py namespace: a200
<p align="right">(<a href="#readme-top">back to top</a>)</p>Roadmap[x] Integrate Clearpath Husky with ROS 2 Humble13131313.[x] Implement LiDAR-based wall tracking logic.[x] Integrate YOLOv8 for human detection.[ ] Optimize Real-Time Factor (RTF) for faster simulation14.[ ] Enhance side-positioning PID controllers15.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Felipe Ramirez Cortes - felipe.cortes@edu.ufes.br

Juan Camilo Castro Rizo - juan.rizo@edu.ufes.br

Juan Sebastian Parra Perdomo - juan.perdomo@edu.ufes.br

Project Link: [https://github.com/your_username/repo_name](https://github.com/your_username/repo_name)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
  * Prof. Ricardo Mello
  * Clearpath Robotics Tutorials
  * Ultralytics YOLOv8 Documentation
  * ROS 2 Humble Documentation

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/Felipe-Ramirez-C/RobosAutonomos_FRC_JCCR_JSPP.svg?style=for-the-badge
[contributors-url]: https://github.com/Felipe-Ramirez-C/RobosAutonomos_FRC_JCCR_JSPP/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/Felipe-Ramirez-C/RobosAutonomos_FRC_JCCR_JSPP.svg?style=for-the-badge
[forks-url]: https://github.com/Felipe-Ramirez-C/RobosAutonomos_FRC_JCCR_JSPP/network/members
[stars-shield]: https://img.shields.io/github/stars/Felipe-Ramirez-C/RobosAutonomos_FRC_JCCR_JSPP.svg?style=for-the-badge
[stars-url]: https://github.com/Felipe-Ramirez-C/RobosAutonomos_FRC_JCCR_JSPP/stargazers
[issues-shield]: https://img.shields.io/github/issues/Felipe-Ramirez-C/RobosAutonomos_FRC_JCCR_JSPP.svg?style=for-the-badge
[issues-url]: https://github.com/Felipe-Ramirez-C/RobosAutonomos_FRC_JCCR_JSPP/issues

[ros2-url]: https://docs.ros.org/en/humble/index.html
[ros2-shield]: https://img.shields.io/ros/v/humble/vision_msgs
[python-url]: https://www.python.org
[python-shield]: https://img.shields.io/badge/python-000000?style=for-the-badge&logo=python&logoColor=white
[gazebo-url]: https://gazebosim.org/docs/latest/getstarted
[gazebo-shield]: https://img.shields.io/badge/gazebo-000000?style=for-the-badge&logo=gazebo&logoColor=white
[yolov8-url]: https://github.com/ultralytics/ultralytics
[yolov8-shield]: https://img.shields.io/badge/yolo-000000?style=for-the-badge&logo=yolo&logoColor=white
