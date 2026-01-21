<a id="readme-top"></a>

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![Unlicense License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

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
[][ROS2-url]

[][Python-url]

[][Gazebo-url]

[][YOLOv8-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
Prerequisites
Ubuntu 22.04 LTS 
ROS 2 Humble Hawksbill 
Gazebo Simulator 
YOLOv8 (Ultralytics)

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
Usage To launch the full simulation, including the environment, robot, and autonomous nodes11:Bashros2 launch your_package_name simulation.launch.py
Known Simulation FixesIf the robot model or TFs do not appear in RViz automatically, run the visualization launch in a separate terminal12121212:Bashros2 launch clearpath_viz view_robot.launch.py namespace: a200
<p align="right">(<a href="#readme-top">back to top</a>)</p>Roadmap[x] Integrate Clearpath Husky with ROS 2 Humble13131313.[x] Implement LiDAR-based wall tracking logic.[x] Integrate YOLOv8 for human detection.[ ] Optimize Real-Time Factor (RTF) for faster simulation14.[ ] Enhance side-positioning PID controllers15.<p align="right">(<a href="#readme-top">back to top</a>)</p>ContactYour Name - your_email@example.comProject Link: https://github.com/your_username/your_repo<p align="right">(<a href="#readme-top">back to top</a>)</p>AcknowledgmentsProf. Ricardo Mello - PPGEE-0552 16Clearpath Robotics Tutorials 17Ultralytics YOLOv8 DocumentationROS 2 Humble Documentation

<p align="right">(<a href="#readme-top">back to top</a>)</p>[]: 

[contributors-url]: https://www.google.com/search?q=%5Bhttps://github.com/your_username/your_repo/graphs/contributors%5D$https://github.com/your_username/your_repo/graphs/contributors
[forks-url]: https://www.google.com/search?q=%5Bhttps://github.com/your_username/your_repo/network/members%5D$https://github.com/your_username/your_repo/network/members
[stars-url]: https://www.google.com/search?q=%5Bhttps://github.com/your_username/your_repo/stargazers%5D$https://github.com/your_username/your_repo/stargazers
[issues-url]: https://www.google.com/search?q=%5Bhttps://github.com/your_username/your_repo/issues%5D$https://github.com/your_username/your_repo/issues
[license-url]: https://www.google.com/search?q=%5Bhttps://github.com/your_username/your_repo/blob/master/LICENSE%5D$https://github.com/your_username/your_repo/blob/master/LICENSE
[ros2-url]: https://www.google.com/search?q=%5Bhttps://docs.ros.org/en/humble/%5D$https://docs.ros.org/en/humble/
[python-url]: https://www.google.com/search?q=%5Bhttps://www.python.org/%5D$https://www.python.org/
[gazebo-url]: https://www.google.com/search?q=%5Bhttps://gazebosim.org/%5D$https://gazebosim.org/
[yolov8-url]: https://www.google.com/search?q=%5Bhttps://github.com/ultralytics/ultralytics%5D$https://github.com/ultralytics/ultralytics$
