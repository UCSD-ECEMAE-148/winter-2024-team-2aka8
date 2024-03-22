<div id="top"></div>

<h1 align="center">TranquiBot: Chatgpt Powered Autonomous Car</h1>
<h4 align="center">We let it choose that name</h4>
<!-- PROJECT LOGO -->:
<br />
<div align="center">
  <a href="https://jacobsschool.ucsd.edu/">
    <img src="UCSDLogo_JSOE_BlueGold.png" alt="Logo" width="400" height="100">
  </a>
<h3>MAE148 Final Project</h3>
<p>
Team 8 Winter 2024
</p>

![image](https://github.com/JL2200/mae148_group8/blob/main/IMG_4898.JPG)
</div>




<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#final-project">Final Project</a></li>
      <ul>
        <li><a href="#original-goals">Original Goals</a></li>
          <ul>
            <li><a href="#goals-we-met">Goals We Met</a></li>
            <li><a href="#our-hopes-and-dreams">Our Hopes and Dreams</a></li>
              <ul>
                <li><a href="#stretch-goal-1">Stretch Goal 1</a></li>
                <li><a href="#stretch-goal-2">Stretch Goal 2</a></li>
              </ul>
          </ul>
        <li><a href="#final-project-documentation">Final Project Documentation</a></li>
      </ul>
    <li><a href="#robot-design">Robot Design </a></li>
      <ul>
        <li><a href="#cad-parts">CAD Parts</a></li>
          <ul>
            <li><a href="#final-assembly">Final Assembly</a></li>
            <li><a href="#custom-designed-parts">Custom Designed Parts</a></li>
            <li><a href="#open-source-parts">Open Source Parts</a></li>
          </ul>
        <li><a href="#electronic-hardware">Electronic Hardware</a></li>
        <li><a href="#software">Software</a></li>
          <ul>
            <li><a href="#embedded-systems">Embedded Systems</a></li>
            <li><a href="#ros2">ROS2</a></li>
            <li><a href="#donkeycar-ai">DonkeyCar AI</a></li>
          </ul>
      </ul>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
    <li><a href="#authors">Authors</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- TEAM MEMBERS -->
## Team Members

<div align="center">
    <p align = "center">Jesse, Jason, Maahir, and Alexander</p>
</div>

<h4>Team Member Major and Class </h4>
<ul>
  <li>Jesse - Mechanical Engineering, Controls and Robotics - Class of 2026</li>
  <li>Jason - Mechanical Engineering - Class of 2025</li>
  <li>Maahir - Mechanical Engineering - Class of 2025</li>
  <li>Alexander - Mechanical Engineering, Controls and Robotics - Class of 2025</li>
</ul>

<!-- Final Project -->
## Final Project

Our project goal was to integrate ChatGPT into the robocar framework. Using chatgpt, the robot can respond to input from the camera, lidar, and gps in order to do different moves and paths.

<!-- Original Goals -->
### Original Goals

   
<!-- End Results -->
### Goals We Met




### Future Goals
#### Stretch Goal 1


#### Stretch Goal 2


### Final Project Documentation


<!-- Early Quarter -->
## Robot Design CAD
<img src="https://github.com/JL2200/mae148_group8/blob/main/full%20car%20cad.png" width="400" height="300" />

#### Open Source Parts
| Part | CAD Model | Source |
|------|--------|-----------|
| Jetson Nano Case | <img src="https://github.com/JL2200/mae148_group8/blob/main/jetson%20nano%20case.png" width="400" height="300" /> | [Thingiverse](https://www.thingiverse.com/thing:3518410) |


### Software
#### Chatgpt
We connected Chatgpt to the robocar by using the Openai API, utilizing two seperate models. The first model was GPT4 with Vision, which processes commands from the user and images. Then this model creates an action plan for what the car can do. Since the image based models don't have function calling to trigger the drive commands, we used a second GPT4-Turbo model to read the vision models plan and turn those into functions. Chatgpt had acces to two functions, a drive command to control steering, speed, and motion timeout. Then it had control over a path function, which let chatgpt generate a csv path of x and y coordinates + a throttle for the car to follow. Chatgpt had access to the cameras, lidar data, gps data, and user prompts. We picked and chose which data to give Chatgpt based on the use case.

#### Embedded Systems
To run the system, we used a Jetson Nano with an Oakd depth camera, an ld06 lidar sensor, and a point one Fusion Engine gps. For motion we used a VESC Driver within the Donkey Car framework. https://www.donkeycar.com/

#### ROS2
For commands, we made a ROS2 Node called 

#### DonkeyCar AI


<!-- Authors -->
## Authors
Jason, Jesse, Maahir, Alexander

<!-- Badges -->
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
*Thank you to my teammates, Professor Jack Silberman, and our incredible TA Arjun Naageshwaran for an amazing Winter 2024 class! Thank you Kiersten for the amazing readme template.*

<!-- CONTACT -->
## Contact

* Jason | yul202@ucsd.edu
* Jesse | jerupe@ucsd.edu 
* Maahir | masgharali@ucsd.edu
* Alexander | ahaken@ucsd.edu
