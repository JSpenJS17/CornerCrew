To run:
```
ros2 run sumo color_tracker
ros2 run maze [maze_right/maze_left]
```

To run with SLAM:
```
ros2 launch maze slam_remap_launch.py
```

To run navigation (run with SLAM also running):
```
ros2 launch maze nav2_remap_launch.py
```

To navigate, use rviz:
```
ros2 launch navigation rviz_navigation.launch.py
```

Navigation was never fully completed because SLAM never produces a good maze map.
