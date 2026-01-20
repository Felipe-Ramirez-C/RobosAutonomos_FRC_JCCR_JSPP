<a id="readme-top"></a>

[][contributors-url] [][forks-url] [][stars-url] [][issues-url] [][license-url]

<br /> <div align="center"> <h3 align="center">Autonomous Human Search and Positioning</h3>

<p align="center"> A ROS 2 Humble project for autonomous navigation, human detection using YOLOv8, and side-positioning with a Clearpath Husky A200. <br /> <a href="https://github.com/your_username/your_repo"><strong>Explore the docs »</strong></a> <br /> <br /> <a href="https://github.com/your_username/your_repo">View Demo</a> · <a href="https://github.com/your_username/your_repo/issues">Report Bug</a> · <a href="https://github.com/your_username/your_repo/issues">Request Feature</a> </p> </div>




<details> <summary>Table of Contents</summary> <ol> <li> <a href="#about-the-project">About The Project</a> <ul> <li><a href="#built-with">Built With</a></li> </ul> </li> <li> <a href="#getting-started">Getting Started</a> <ul> <li><a href="#prerequisites">Prerequisites</a></li> <li><a href="#installation">Installation</a></li> </ul> </li> <li><a href="#usage">Usage</a></li> <li><a href="#roadmap">Roadmap</a></li> <li><a href="#contact">Contact</a></li> <li><a href="#acknowledgments">Acknowledgments</a></li> </ol> </details>

About The Project
This project addresses the challenge of autonomous exploration and human interaction in a complex "office" environment. The robot, a Clearpath Husky A200, must autonomously navigate a partially unknown space to find a person, signal their location, and position itself precisely next to them.





Key features:


100% Autonomy: No teleoperation or external commands during the task.

Intelligent Perception: Utilizes YOLOv8 for human classification and pose identification.

Wall-Following Logic: Custom LiDAR-based navigation for environment exploration.


Automated Reporting: Generates a log file with the person's location and total mission time.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

Built With
[][ROS2-url]

[][Python-url]

[][Gazebo-url]

[][YOLOv8-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

Getting Started
Prerequisites

Ubuntu 22.04 LTS 



ROS 2 Humble Hawksbill 



Gazebo Simulator 

YOLOv8 (Ultralytics)

Installation
Install Clearpath Husky simulation packages:

