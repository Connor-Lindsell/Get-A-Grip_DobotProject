# Get-A-Grip: Control and Grasping for the DoBot Robot

## Project Overview
This project, **Get-A-Grip**, focuses on the implementation of a vision-guided robotic manipulation system using the **DoBot Magician robotic arm** integrated with an **RGB-D camera**. The objective is to enable the robot to autonomously identify, pick, and stack coloured cubes within a defined workspace. The RGB-D camera provides both colour and depth information, allowing for object recognition, 3D position estimation, and vision-based control of the robot.

The project combines concepts from sensing, control systems, and robotics—specifically **image-based visual servoing (IBVS)**—to create a closed-loop control system that can accurately position the robot based on visual feedback.

---

## Objectives
- Integrate the RGB-D sensor with the DoBot robotic arm using ROS2.
- Implement colour segmentation and depth estimation algorithms to detect and localise objects.
- Calibrate the camera with respect to the robot base for accurate coordinate transformation.
- Develop and test a visual servoing control algorithm (IBVS) to guide the robot’s end effector toward detected targets.
- Implement safe motion planning for movement within joint and workspace limits.
- Enable fully automated pick-and-place operations to grasp and stack objects based on their colour classification.
- Conduct integration testing and evaluate the system’s accuracy, stability, and repeatability.

---

## Project Scope
The project scope is limited to static, pre-coloured cubic objects under controlled lighting conditions. The system will:
- Detect three distinct coloured cubes placed randomly within the workspace.
- Estimate their 3D positions using RGB-D data.
- Pick and stack the cubes by colour into vertical towers.

Advanced tasks such as moving object tracking, dynamic grasp planning, or complex vision-based manipulation are outside the scope due to time constraints.

---

## Deliverables
1. **RGB-D Sensor Integration**  
   - Establish data acquisition from the RGB-D camera and visualise output for verification.
2. **Vision-Based Control Logic (IBVS)**  
   - Develop algorithms to map visual information to robot movement commands.
3. **Movement to Goal Poses**  
   - Implement motion control that respects kinematic limits and workspace constraints.
4. **Pick-and-Place Functionality**  
   - Design and test grasping sequences for autonomous manipulation.
5. **System Integration and Testing**  
   - Combine perception, control, and actuation into a unified ROS2-based framework.
6. **Demonstration and Documentation**  
   - Provide a final working demonstration and report detailing methods, setup, and results.

---

## Proposed Implementation Approach
The project follows a staged development process:
1. **Sensor Data Acquisition:** Set up the RGB-D sensor and validate colour and depth outputs.  
2. **Perception and Classification:** Implement image segmentation to identify coloured cubes and determine their positions.  
3. **Control and Visual Servoing:** Develop IBVS logic to translate camera feedback into precise robot movements.  
4. **Path Planning and Pick-and-Place:** Implement trajectory generation and grasping sequences.  
5. **Integration and Testing:** Merge all components into a single working system, followed by extensive testing and debugging.  
6. **Documentation and Presentation:** Prepare the final report and demonstration video showcasing project outcomes.

---

## Team Responsibilities
| Member | Role | Primary Responsibilities |
|---------|------|---------------------------|
| **Elijah Spannenberg** | Project Lead | Lead developer for IBVS control and system coordination |
| **Luca Sanchez** | Motion Lead | Development of motion planning, goal movement logic, and documentation |
| **Riley Sheridan** | Perception Lead | RGB-D sensor integration and data processing; testing assistant |
| **Connor Lindsell** | Manipulation Lead | Pick-and-place functionality, gripper control, and testing lead |

---

## Tools and Resources
- **Framework:** ROS2 with the Dobot ROS2 SDK  
- **Programming Languages:** Python and C++  
- **Version Control:** GitHub (collaborative repository for code management)  
- **Hardware:** DoBot Magician robotic arm, RGB-D camera, coloured cube samples  
- **Testing Environment:** UTS Robotics Laboratory  

Reference SDK: [Dobot-Arm/DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/blob/main/README_EN.md)

---

## Assessment Focus
The project will be evaluated based on:
- Functionality and reliability of path planning and collision avoidance.
- Accuracy of RGB-D sensor integration and colour detection.
- Stability and performance of robot trajectory execution.
- Success and consistency of pick-and-place operations.
- Clarity and completeness of documentation and system presentation.

---

## Learning Outcomes
- Design and implement a vision-based robotic control system.
- Integrate sensing, actuation, and control within a ROS2 framework.
- Apply mechatronic principles to achieve autonomous manipulation.
- Collaborate effectively using version control and modular system design.
