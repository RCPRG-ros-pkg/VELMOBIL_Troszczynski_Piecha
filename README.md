# VELMOBIL_Troszczynski_Piecha
*Authors:*    [**Kamil Troszczyński**](https://github.com/Kamil-Troszczynski), [**Miłosz Piecha**](https://github.com/Coffee4Cat)
## Description:
This repository contains code used for Velmobil autonomous traversal utilizing Reinforcement Learning. 

## Prerequisites:
It's best to run the code with CUDA support.  
What to install:   
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)  
- [StableBaselines3](https://stable-baselines3.readthedocs.io/en/master/guide/install.html)   
- [GZ Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)

## How to launch simulation?
```bash
#   Clone repository to get source files
git clone git@github.com:RCPRG-ros-pkg/VELMOBIL_Troszczynski_Piecha.git

#   Paste it in current directory
source /opt/ros/humble/setup.bash

#   Go to script
cd src/velmobil/scripts

#   Give execution permission
chmod +x sim_launcher.sh

#   Run it
./sim_launcher.sh
```