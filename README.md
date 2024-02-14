# Labs
Lab assignments from F1TENTH 
https://github.com/f1tenth/f1tenth_labs_openrepo

# F1TENTH Gym ROS - Install Guide
## Prerequisites for Windows 10/11
- [Docker Desktop](https://www.docker.com/products/docker-desktop/)
- [Git](https://git-scm.com/downloads)
## Windows 10 (Optional)
[Windows Terminal](https://apps.microsoft.com/detail/9N0DX20HK701?hl=en-us&gl=US)

This is not required but may provide a better experience with tabs.

Change the default Windows Console to the new Windows Terminal in `Settings -> Default terminal application -> Windows Terminal`
![image](https://github.com/CPP-F1TENTH-SDP/Labs/assets/135196190/eee20ae6-ea21-4b4d-adf1-49a7aedb8c04)
Now whenever opening PowerShell, it will use the new Windows Terminal
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
5. Start Docker Desktop, open PowerShell/Terminal and run:
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
   This command is based on the assumption that the files in Step 5, are in `C:\Users\<USERNAME>\f1tenth_gym_ros`.

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
   
