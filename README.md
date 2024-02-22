### Table of Contents
1. [Labs](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#labs)
2. [Non-NVIDIA Node Launch](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#non-nvidia-node-launch)
3. [NVIDIA Node Launch](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#nvidia-node-launch)
4. [Keyboard Control](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#keyboard-control)
5. [Lab 2 - safety_node](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#lab-2---safety_node)
6. [Lab 3 - wall_follow](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#lab-3---wall_follow)
7. [Lab 4 - gap_follow](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#lab-4---gap_follow)
8. [F1TENTH Gym ROS - Install Guide](https://github.com/CPP-F1TENTH-SDP/Labs/tree/main?tab=readme-ov-file#f1tenth-gym-ros---install-guide)

---

# Labs
Lab assignments from [F1TENTH](https://github.com/f1tenth/f1tenth_labs_openrepo).

## Non-NVIDIA Node Launch
1. Download `<node>`.zip and unzip the folder.
2. Launch the simulation. ([Non-NVIDIA Launch](https://github.com/CPP-F1TENTH-SDP/Labs/wiki/F1TENTH-Gym-ROS-%E2%80%90-Install-Guide#non-nvidia-launch))
3. In Docker Desktop, go to sim 1 container and click on the vertical ellipsis, then "View Files". Under "Files", find the folder `sim_ws`.
4. In `sim_ws`, there is a folder named `src`. Drag & drop the unzipped <node> folder into the `src` folder.

   **Be sure that you are dragging the _folder_ into `src` and not the contents of the folder.**
5. Open another PowerShell/Terminal and run:
   ```
   source /opt/ros/foxy/setup.bash
   ```
   ```
   source install/local_setup.bash
   ```
   ```
   colcon build
   ```
6. Rerun:
   ```
   source /opt/ros/foxy/setup.bash
   ```
   ```
   source install/local_setup.bash
   ```
7. Then, run `<node>` using:
   ```
   ros2 run <node> <node>
   ```
   
## NVIDIA Node Launch
**_Note: Docker container self-destructs after closing sim, you'll have to drag & drop the `<node>` folder into `src` each time._**
1. Download `<node>`.zip and unzip the folder.
2. Open a PowerShell/Terminal and follow [NVIDIA Launch](https://github.com/CPP-F1TENTH-SDP/Labs/wiki/F1TENTH-Gym-ROS-%E2%80%90-Install-Guide#nvidia-launch), **STOP** after:
   ```
   rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros/Users/<USERNAME>/f1tenth_gym_ros -- f1tenth_gym_ros
   ```
3. Run:
   ```
   tmux
   ```
4. After `tmux`, hit `CTRL + B, then C`. You now have 2 bash instances.
   _Note: For tmux, don't use CTRL for the key after 'then'. More commands visit: https://tmuxcheatsheet.com/_
5. Hit `CTRL + B, then P` (it should be in bash 0). Launch the simulation using:
   ```
   source /opt/ros/foxy/setup.bash
   ```
   ```
   source install/local_setup.bash
   ```
   ```
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```
6. In Docker Desktop, go to the active container and click on the vertical ellipsis, then "View Files". Under "Files", find the folder `sim_ws`.
7. In `sim_ws`, there is a folder named `src`. Drag & drop the unzipped <node> folder into the `src` folder.
    
   **Be sure that you are dragging the _folder_ into `src` and not the contents of the folder.**
8. Hit `CTRL + B, then N` (it should be in bash 1) & run:
   ```
   source /opt/ros/foxy/setup.bash
   ```
   ```
   source install/local_setup.bash
   ```
   ```
   colcon build
   ```
9. Rerun:
   ```
   source /opt/ros/foxy/setup.bash
   ```
   ```
   source install/local_setup.bash
   ```
10. Then, run `<node>` using:
    ```
    ros2 run <node> <node>
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

## Lab 2 - safety_node
For Node Launch above, `<node>` is replaced with `safety_node`.

Download safety_node.zip from [Releases](https://github.com/CPP-F1TENTH-SDP/Labs/releases/tag/lab2-v1.0) or [Direct Download](https://github.com/CPP-F1TENTH-SDP/Labs/releases/download/lab2-v1.0/safety_node.zip)

Run using:
```
ros2 run safety_node safety_node
```

## Lab 3 - wall_follow
For Node Launch above, `<node>` is replaced with `wall_follow`.

Download wall_follow.zip from [Releases](https://github.com/CPP-F1TENTH-SDP/Labs/releases/tag/lab3-v1.0) or [Direct Download](https://github.com/CPP-F1TENTH-SDP/Labs/releases/download/lab3-v1.0/wall_follow.zip).

Run using:
```
ros2 run wall_follow wall_follow
```

## Lab 4 - gap_follow
⚠️**INCOMPLETE**⚠️

Download gap_follow.zip from [Releases](https://github.com/CPP-F1TENTH-SDP/Labs/releases/tag/lab4-v1.0) or [Direct Download](https://github.com/CPP-F1TENTH-SDP/Labs/releases/download/lab4-v1.0/gap_follow.zip).

Run using:
```
ros2 run gap_follow reactive_node
```

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
The install/launch guide is [here](https://github.com/CPP-F1TENTH-SDP/Labs/wiki/F1TENTH-Gym-ROS-%E2%80%90-Install-Guide).
