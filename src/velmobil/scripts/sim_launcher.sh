cd ../../..
colcon build --packages-select velmobil
source install/setup.bash
ros2 launch velmobil simple_velmobil.launch.py
