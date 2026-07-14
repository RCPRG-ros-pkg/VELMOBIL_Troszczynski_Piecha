# VELMOBIL_Troszczynski_Piecha
<p align="left">
  <img src="https://img.shields.io/badge/ROS-HUMBLE-blue?style=plastic">
  <img src="https://img.shields.io/badge/Ignition-Fortress-orange?style=plastic">
  <img src="https://img.shields.io/badge/Stable-Baselines3-blue?style=plastic">
</p> 

Authors:
- **Kamil Troszczyński** ([Github](https://github.com/Kamil-Troszczynski), [LinkedIn](https://www.linkedin.com/in/kamil-troszczy%C5%84ski-a962a538a/))
- **Miłosz Piecha** ([Github](https://github.com/Coffee4Cat), [LinkedIn](https://www.linkedin.com/in/mi%C5%82osz-piecha-06bb20387/))
## Description:
This repository contains code used for Velmobil autonomous traversal utilizing Reinforcement Learning. 

## Prerequisites:
It's best to run the software with CUDA support.  
What to install:   
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)  
- [Ignition Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)


## Guidelines
### Building Project
1. Clone repository and build docker image
```bash
git clone https://github.com/RCPRG-ros-pkg/VELMOBIL_Troszczynski_Piecha.git
```

```bash
cd VELMOBIL_Troszczynski_Piecha
```

```bash
docker build -t velmobil_simulation:latest .
```

2. Add permissions to bash script in order to run container and run it
```bash
chmod +x docker_launcher.sh
```

```bash
./docker_launcher.sh
```

3. Get into container
```bash
docker exec -it velmobil_simulation bash
```

4. Build and source
```bash
colcon build
```

```bash
source /root/ws/install/setup.bash
```

### Running simulation
Bare simulation.
```bash
ros2 launch velmobil_simulation simple_velmobil.launch.py
```

### How to run training node
```bash
ros2 run rl_tools training_node
```

### How to control velmobil
#### In a new terminal attached to the running container, execute either the first or the second command.
First option:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard repeat_rate:=50
```

Second option:
```bash
ros2 topic pub  /cmd_vel geometry_msgs/msg/Twist "{linear: {x: <velocity_x>, y: <velocity_y>}, angular: {z: <angular_velocity>}}"
```

### How to exit and stop container
```bash
exit
```
```bash
docker stop velmobil_simulation
```

## Github Project
[Here](https://github.com/orgs/RCPRG-ros-pkg/projects/22/views/1) is our Task Board used for progress control/dev logs.


## Future Dev Notes
1. We will provide docs/dev_logs




