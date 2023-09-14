
```

### Build and run Gazebo simulation

Building:
```
cd workspace_folder
colcon build
source install/setup.bash
```

Running:
```
ros2 launch rosbot_gazebo demo.launch.py 
ros2 launch rosbot_gazebo nav2.launch.py
ros2 launch frontier_detector frontiers.launch.py 
ros2 launch decision_maker autonomous_agent.launch.py 
```


