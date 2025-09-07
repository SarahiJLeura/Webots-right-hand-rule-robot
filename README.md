\# Project - Robotics Programming (Partial Exam 3)



This repository contains the implementation of the \*\*Third Partial Exam\*\* for the \*Robotics Programming\* course using \*\*Webots\*\*.



\## 📌 Description

The project implements the \*\*"Blind Maze"\*\* scenario in Webots:

\- The robot cannot rely on visual input.

\- The robot is an e-puck

\- Instead, it must use its \*\*proximity sensors\*\* to navigate and avoid collisions.

\- The navigation is based on the \*\*Right-Hand Rule\*\*:  

&nbsp; The robot will always follow the nearest wall on its right-hand side, regardless of the number of turns, until it finds the exit.

\- When the robot reaches the goal coordinates it stops.



\## ⚙️ Controller

The robot controller implements the following logic:

\- Continuously reads \*\*proximity sensors\*\*.

\- Identifies the right wall and moves alongside it.

\- If there is an opening on the right, the robot will turn right.

\- If the right side is blocked, the robot will keep moving forward.

\- If both front and right are blocked, the robot will turn left.

\- The goal is to \*\*exit the maze\*\* autonomously



The controller code is fully documented:

\- Each variable and function is explained.

\- Sections of code that were adapted from examples are clearly indicated.



\## 📁 Repository Structure

\- `worlds/Parcial3.wbt` → Maze world with sand floor and brick walls.

\- `controllers/right\_hand\_rule/right\_hand\_rule.cpp` → Robot controller (with comments).

\- `worlds/textures/` → Textures for floor and walls.



\## ▶️ How to Run

1\. Open the project in \*\*Webots\*\*.

2\. Load the world `Parcial3.wbt`.

3\. Run the controller `right\_hand\_controller.py` (Remove intermediate build files and build the current project).

4\. Observe how the robot follows the right-hand rule until it exits the maze.



\## 🎥 Demo Video

The simulation video can be found at the following link:  

👉 \* https://youtu.be/fnaHWV5M-4s \*



---



✍️ \*\*Author:\*\* \* Patricia Sarahi Jimenez-Leura \*  

📅 \*\*Due date:\*\* May 27, 2025



