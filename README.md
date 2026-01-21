# SLAM using Robile

This repository serves as the **main landing page** for the Robile project.
It provides a high-level overview and links to individual repositories / branches for each task.

Each task was developed separately to keep the code clean and organized. Use the links below to quickly navigate to each task.


## Working Video : 

[![Working Video](https://img.youtube.com/vi/AGzE0Z4cm9Y/0.jpg)](https://www.youtube.com/watch?v=AGzE0Z4cm9Y)

---

## 📚 Tasks

| Task       | Description                                                                 | Link                                                                    |
| ---------- | --------------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| **Task 1** | A\* Global Planner + Potential Field Planner combined for Robile navigation | [🔗 Task 1 Branch](https://github.com/Nikhil-URG/Autonomous-mobile-robot-project/tree/task-1) |
| **Task 2** | *(A\* Global Planner + Potential Field Planner+ Monte Carlo Localization combined for Robile navigation)*                                 | [🔗 Task 2 Branch](https://github.com/Nikhil-URG/Autonomous-mobile-robot-project/tree/task-2) |
| **Task 3** | Automated Mapping & Environment Exploration                                 | [🔗 Task 3 Branch](https://github.com/Nikhil-URG/Autonomous-mobile-robot-project/tree/task-3) |

---

## 🚀 How to Use

* Click on any of the links above to go directly to the repository/branch for that task.
* Each task repository has its own **README.md** with full setup and run instructions.
* Clone and build the task repositories individually following their instructions.

---

## 📝 Notes

* All tasks are ROS 2 based and tested with Robile.
* The separation ensures modularity and easy navigation.
* Use the provided README files in each repository to set up and run the systems.






[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/pDOoIxCj)
# AMR Project

## Project Objectives

The objective of this project is that you deploy some of the functionalities that were discussed during the course on a real robot platform. In particular, we want to have functionalities for path and motion planning, localisation, and environment exploration on the robot.

We will particularly use the Robile platform during the project; you are already familiar with this robot from the simulation you have been using throughout the semester as well as from the few practical lab sessions that we have had.

## Task Description

The project consists of three parts that are building on each other: (i) path and motion planning, (ii) localisation, and (iii) environment exploration.

### 1. Path and Motion Planning

You have already implemented a *potential field planner* in one of your assignments. In this first part of the project, you need to port your implementation to the real robot and ensure that it is working as well as it was in the simulated environment so that you can navigate towards global goals while avoiding obstacles. Then, integrate your potential field planner with a global path planner, namely first use a path planner (e.g. A*) to find a rough global trajectory of waypoints that the robot can follow to reach a goal and then use the potential field planner to navigate between the waypoints. This will make your potential field planner applicable to large environments, where it can navigate given an environment map.

### 2. Localisation

In one of the course lectures, we discussed Monte Carlo localisation as a practical solution to the robot localisation problem in an existing map. In this second part of the project, your objective is to implement your very own particle filter that you then integrate on the Robile. You should implement the simple version of the filter that we discussed in the lecture; however, if you have time and interest, you are free to additionally explore extensions / improvements to the algorithm, for example in the form of the adaptive Monte Carlo approach that we mentioned in the lecture.

### 3. Environment Exploration

The final objective of the project is to incorporate an environment exploration functionality to the robot. This will have to be combined with a SLAM component, namely you will need your exploration component to select poses to explore and a SLAM component that will take care of actually creating a map. The exploration algorithm should ideally select poses at the map fringe (i.e. poses that are at the boundary between the explored and unexplored region), but you are free to explore different pose selection strategies in your implementation.
