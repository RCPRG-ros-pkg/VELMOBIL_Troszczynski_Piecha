# VELMOBIL_Troszczynski_Piecha
![](https://img.shields.io/badge/ROS-HUMBLE-blue?style=plastic
)  
Authors:
- **Kamil Troszczyński** ([Github](https://github.com/Kamil-Troszczynski), [LinkedIn](https://www.linkedin.com/in/kamil-troszczy%C5%84ski-a962a538a/))
- **Miłosz Piecha** ([Github](https://github.com/Coffee4Cat), [LinkedIn](https://www.linkedin.com/in/mi%C5%82osz-piecha-06bb20387/))
## Description:
This repository contains code used for Velmobil autonomous traversal utilizing Reinforcement Learning. 

## Prerequisites:
It's best to run the software with CUDA support.  
What to install:   
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)  
- [StableBaselines3](https://stable-baselines3.readthedocs.io/en/master/guide/install.html)   
- [GZ Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)




## Guidelines
### Building Project
1. Clone repository
```bash
source /opt/ros/humble/setup.bash
git clone https://github.com/RCPRG-ros-pkg/VELMOBIL_Troszczynski_Piecha.git
cd VELMOBIL_Troszczynski_Piecha
```
2. Install dependencies
```bash
chmod +x dependencies_install.sh
./dependencies_install.sh
```
3. Build and source
```bash
colcon build
source instal/setup.bash
```

### Running simulation
Bare simulation.
```bash
ros2 launch velmobil_simulation simple_velmobil.launch.py
```




## Future Dev Notes
1. We will provide docs/dev_logs
2. Docker might be a future option. At the moment, *dependencies_install.sh* is sufficient.




