# ENPM 661 Project 3 Phase 2 : TurtleBot A star Planner
#### Project Members : Adil Qureshi (1220607905), Khuzema Habib(12188776), Sounderya Venugopal(121272423)

##  Project Description

In this project, we use an A star Path Planner to generate a trajectory for a given map in 3 different environments:

- 2D Environment
- Gazebo
- Falcon Simulator

We first run the 2D Path Planner to generate a trajectory for the other 2 simulations in 3D, applying clearances where required. The 2D Map was generated with the use of half planes and semi-algebraic equations in Matplotlib then visualized in opencv for trajectory generation. 

Link to Folder Containing Path Generation, Gazebo and Falcon Simulations can be found here : https://drive.google.com/drive/folders/1ESFB4iaVE5oencMvhOxJujY2T5UsuXSq?usp=drive_link

## 2D Map with Clearances Applied


![image](https://github.com/user-attachments/assets/14151b3c-c3fe-4c75-966d-a0b403a5545d)


## 2D Map with Generated Path

![image](https://github.com/user-attachments/assets/7f23e4b0-3b3d-4468-9dac-325c41fc2615)

## 3D Map in Gazebo

![gazebo](https://github.com/user-attachments/assets/44055b91-de16-472b-9eb6-26878d8b54f4)

## 3D Map in Falcon Simulator

![image](https://github.com/user-attachments/assets/a202a682-5a2d-4597-a815-1ee8281674fa)


# Running Code

## Clone the Repository


```sh
git clone https://github.com/khuzema-h/Turtlebot_3_A_star_Path_Finder.git
cd ~/Turtlebot_3_A_star_Path_Finder
```

## Build and Source Workspace

```sh
cd ~/Turtlebot_3_A_star_Path_Finder
colcon build
source install/setup.bash
```
## Run 2D Path Planner

```sh
cd ~/Turtlebot_3_A_star_Path_Finder/src/turtlebot_project3
python3 astar.py
```
## Suggested Values for User Input:

Robot Clearance : 30
Start Coordinates : 220, 1500, 30
Goal Coordinates : 5000,1500
RPM1 : 5
RPM2 : 10

Sample Outout: 


```sh
Enter Robot Clearance (in millimeters 10 - 40) : 30
Enter start x (in millimeters): 220
Enter start y (in millimeters): 1500
Enter start theta (in degrees): 30
Enter goal x (in millimeters): 5000
Enter goal y (in millimeters): 1500
Enter RPM1 (1 - 10): 5
Enter RPM2 (10 - 20): 10
Generating Path....

```

# Run Gazebo Simulation 

## Build and Source Workspace

```sh
cd ~/Turtlebot_3_A_star_Path_Finder
colcon build
source install/setup.bash
```

## Launch Gazebo Competition World 

```sh
ros2 launch turtlebot_project3 competition_world.launch.py
```

## Run Trajectory Controller

In a separate terminal run the following command: 

```sh
ros2 run turtlebot_project3 trajectory_controller.py
```








