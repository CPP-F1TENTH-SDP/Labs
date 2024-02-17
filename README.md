### Table of Contents
1. [Labs](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#labs)
2. [Lab 2 - safety_node](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#lab-2---safety_node)
3. [Lab 3 - wall_follow_node](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#lab-3---wall_follow_node)
4. [Lab 4](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#lab-4)
5. [F1TENTH Gym ROS - Install Guide](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#f1tenth-gym-ros---install-guide)
6. [Linux](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#linux)
7. [Prerequisites for Windows 10/11](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#prerequisites-for-windows-1011)
8. [Windows Terminal (Optional)](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#windows-terminal-optional)
9. [Non-NVIDIA Install](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#non-nvidia-install)
10. [Non-NVIDIA Launch](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#non-nvidia-launch)
11. [NVIDIA GPU](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#nvidia-gpu)
12. [NVIDIA Launch](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#nvidia-launch)
13. [Keyboard Control](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#keyboard-control)

---

# Labs
Lab assignments from [F1TENTH](https://github.com/f1tenth/f1tenth_labs_openrepo).

## Lab 2 - safety_node
1. Download safety_node.zip and unzip the folder.
   
   [Releases](https://github.com/CPP-F1TENTH-SDP/Labs/releases/tag/lab2) or [Direct Download](https://github.com/CPP-F1TENTH-SDP/Labs/releases/download/lab2/safety_node.zip)
2. Launch the simulation.
3. Docker - Go to sim 1 container and click on the vertical ellipsis then "View Files".

   Rocker - Go to the active container and click on the vertical ellipsis then "View Files".
4. Under "Files", find the folder `sim_ws`.
5. In `sim_ws`, there is a folder named `src`.
   Drag & drop the unzipped safety_node folder into the `src` folder.
   
   **Be sure that you are dragging the _folder_ into `src` and not the contents of the folder.**
6. Open another PowerShell/Terminal and run:
   ```
   source /opt/ros/foxy/setup.bash
   ```
   ```
   source install/local_setup.bash
   ```
   ```
   colcon build
   ```
7. Run safety_node using:
   ```
   ros2 run safety_node safety_node
   ```
   
## Lab 3 - wall_follow_node
Download wall_follow_node.zip from [Releases](https://github.com/CPP-F1TENTH-SDP/Labs/releases/tag/lab3) or [Direct Download](https://github.com/CPP-F1TENTH-SDP/Labs/releases/download/lab3/wall_follow_node.zip).

Follow the same steps as [Lab 2 - safety_node](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#lab-2---safety_node) and run:
```
ros2 run wall_follow wall_follow
```

## Lab 4
### Changing Map
Lab #4 requires a different map compared to the one we use. You can download the new maps on the github repository and place it inside the f1tenth_gym_ros/maps folder. The map has two files for it, make sure to download both.

1. Find your folder directory that you git cloned the f1tenth_gym_ros folder.

   By default this is in:
   ```
   C:\Users\<USERNAME>\f1tenth_gym_ros\maps
   ```
   Drag & drop the map files into the `maps` folder.
2. In the `f1tenth_gym_ros\config` folder, edit the `sim.yaml` file.

   Use either Notepad or Visual Studio Code to edit it.
3. Locate the maps directory in the config file:
   ```
   # map parameters
   map_path: '/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_map'
   map_img_ext: '.png'
   ```
   Edit the "map_path" line to change the map you want to use.

   Ex: To change the Spielberg_map back to levine, edit the last part of "map_path".
   ```
   map_path: '/sim_ws/src/f1tenth_gym_ros/maps/levine'
   ```
4. Have Docker Desktop running and enter the f1tenth container using PowerShell/Terminal.

   Then rebuild the f1tenth simulation with the new map:
   ```
    colcon build
   ```
5. Launch the simulation and the map should be updated to the new one.

   Use this before launch:
   ```
   source /opt/ros/foxy/setup.bash
   ```
   ```
   source install/local_setup.bash
   ```
❗ Issues:

Map does not change after editing `sim.yaml`.

You may be editing a different .yaml file. 

To edit the correct one, in Docker Desktop, view files of active container, go to source, build, f1tenth gym ros, and config. In the `config` folder there is the `sim.yaml` file, edit it and repeat steps 5 & 6. If the lidar sensors (colored walls) is different than the map outline, restart your docker simulator.

---

# F1TENTH Gym ROS - Install Guide

## Linux
For those on Linux, refer to the official f1tenth_gym_ros guide.

https://github.com/f1tenth/f1tenth_gym_ros

## Prerequisites for Windows 10/11
Download:
- [Docker Desktop](https://www.docker.com/products/docker-desktop/)
- [Git](https://git-scm.com/downloads)

## Windows Terminal (Optional)
This is not required but may provide a better experience with tabs.

Windows Terminal download link: https://apps.microsoft.com/detail/9N0DX20HK701?hl=en-us&gl=US

Change the default Windows Console to the new Windows Terminal in `Settings > Default terminal application > Windows Terminal`.
![image](https://github.com/CPP-F1TENTH-SDP/Labs/assets/135196190/eee20ae6-ea21-4b4d-adf1-49a7aedb8c04)
Now whenever opening PowerShell, it will use the new Windows Terminal.

## Non-NVIDIA Install
1. Open PowerShell/Terminal and run:
   ```
   git clone https://github.com/f1tenth/f1tenth_gym_ros
   ```
2. ```
   cd f1tenth_gym_ros
   ```
3. Have Docker Desktop running:
   ```
   docker compose up
   ```

## Non-NVIDIA Launch
1. Have Docker Desktop running:
   ```
   docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
   ```
2. ```
   source /opt/ros/foxy/setup.bash
   ```
   ```
   source install/local_setup.bash
   ```
   ```
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```
3. Go to: http://localhost:8080/vnc.html

## NVIDIA GPU
1. This will install WSL and Ubuntu. The install time for Ubuntu will vary depending on internet speed. After Ubuntu is installed, you will be prompted to restart the system.
   ```
   wsl --install
   ```
2. Just like step 1, the install of Ubuntu-20.04 will vary with internet speed.
   ```
   wsl.exe --install Ubuntu-20.04
   ```
3. ```
   wsl --set-default Ubuntu-20.04 2
   ```
   Check if Ubuntu-20.04 is default (star before Ubuntu-20.04) using:
   ```
   wsl -l -v
   ```
   If Ubuntu-20.04 is "Stopped", restart Docker Desktop.
   
   ![image](https://github.com/CPP-F1TENTH-SDP/Labs/assets/135196190/25226792-df76-4f8b-b27e-a302d4b0eb05)

5. Start Docker Desktop (if it isn't already), open PowerShell/Terminal and run:
   ```
   git clone https://github.com/f1tenth/f1tenth_gym_ros
   ```
   ```
   cd f1tenth_gym_ros
   ```
   ```
   docker build -t f1tenth_gym_ros -f Dockerfile .
   ```
6. Open another window/tab of PowerShell/Terminal and run:
   ```
   bash
   ```
   ```
   sudo apt-get install python3-venv
   ```
   ⚠️ If python3-venv is not being installed try:
   ```
   sudo apt-get update
   ```
   ```
   sudo apt-get install libpython3-dev
   ```
   ```
   sudo apt-get install python3-venv
   ```
   After python3-venv is installed:
   ```
   mkdir -p ~/rocker_venv
   ```
   ```
   python3 -m venv ~/rocker_venv
   ```
   ```
   cd ~/rocker_venv
   ```
   ```
   . ~/rocker_venv/bin/activate
   ```
   ```
   pip install wheel
   ```
   ```
   pip install git+https://github.com/osrf/rocker.git
   ```
   The installation is now complete.

## NVIDIA Launch
1. Have Docker Desktop running:
   ```
   bash
   ```
   ```
   . ~/rocker_venv/bin/activate
   ```
   This command is based on the assumption that the files from `git clone https://github.com/f1tenth/f1tenth_gym_ros`, are in `C:\Users\<USERNAME>\f1tenth_gym_ros`.

   Replace `<USERNAME>` with your Windows username. You can find this by opening PowerShell/Terminal.
   ```
   rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros/Users/<USERNAME>/f1tenth_gym_ros -- f1tenth_gym_ros
   ```
3. ```
   source /opt/ros/foxy/setup.bash
   ```
   ```
   source install/local_setup.bash
   ```
   ```
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```
   
## Keyboard Control
1. Have the simulation launched and running.
2. Open another window/tab of PowerShell/Terminal and run:
   ```
   source /opt/ros/foxy/setup.bash
   ```
   ```
   source install/local_setup.bash
   ```
   ```
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
2. To use the keyboard, make sure that the window/tab from step 2 is focused.

   2 ways to do this is:
   1. Split screen the PowerShell/Terminal and simulation
      
      OR
   
   2. Maximize the simulation and move the terminal to a corner.
   
