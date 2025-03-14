## Useful Commands:

```
# To build:
cd ~/ros2_ws
colcon build --symlink-install --packages-select <pkg_name>
source ~/ros2_ws/install/setup.zsh

# View topics:
ros2 topic list
rviz2

# Run Sumo:
    # start with this ahead of time:
    ros2 run sumo color_tracker

    # when ready to start, run this in A DIFFERENT terminal window:
    ros2 run sumo sumo
```