# Instruction

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
ros2 launch velmobil_simulation simple_velmobil.launch.py start_pos_x:=-10.0 start_pos_y:=10.0 rviz:=False realsense:=False headless:=True
```

### How to launch simulation manager node with training node
Training and simulation parameters are set in `config/training_params.yaml`. Set `start_pos_x, start_pos_y` parameters in 
file `SimulationManager.py` in order to get equal values with initial position.
```bash
ros2 launch velmobil_agent training.launch.py
```

### How to launch inference
In order to test particular trained policy, go to `velmobil_agent/inference/agent_inference.py` file and change `MODEL_LOAD_PATH` parameter.
```bash
ros2 launch velmobil_agent inference.launch.py
```

### How to draw plots
Drawing plots can be done via command:
```bash
python3 src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/velmobil_agent/common/visualization.py
```

### How to control velmobil

#### Using floating controller

##### In a new terminal attached to the running container, execute either the first or the second command.
First option:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard repeat_rate:=50
```

Second option:
```bash
ros2 topic pub  /cmd_vel geometry_msgs/msg/Twist "{linear: {x: <velocity_x>, y: <velocity_y>}, angular: {z: <angular_velocity>}}"
```

#### Using roll_yaw controller

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