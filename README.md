# VELMOBIL_Troszczynski_Piecha
<p align="left">
  <img src="https://img.shields.io/badge/ROS-HUMBLE-blue?style=plastic">
  <img src="https://img.shields.io/badge/Gazebo-Harmonic-orange?style=plastic">
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
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)


## Program structure

The program structure is documented as SysML diagrams that follow the
[MeROS](https://github.com/twiniars/meros) convention (a SysML-based metamodel for ROS-based systems).
You can view and edit the diagrams in Visual Paradigm.

### Installing Visual Paradigm

1. Download the installer from the [Visual Paradigm download page](https://www.visual-paradigm.com/download/).

2. Go to the directory where the installer was saved, make it executable, and run it.

3. Follow the setup wizard to finish the installation.

### Opening the diagrams

Start Visual Paradigm, choose **Project → Open**, and select
[`velmobil_diagrams/VELMOBIL.vpp`](velmobil_diagrams/VELMOBIL.vpp).
All diagrams are listed in the **Diagram Navigator** and can be edited there.

## Documentation
See [DOCUMENTATION.md](DOCUMENTATION.md) for the repository structure, ROS 2 packages, configuration etc.

## Instruction
See [INSTRUCTION.md](INSTRUCTION.md) for how to run the simulation, training and inference, and how to drive VELMOBIL.

## Github Project
[Here](https://github.com/orgs/RCPRG-ros-pkg/projects/22/views/1) is our Task Board used for progress control/dev logs.



