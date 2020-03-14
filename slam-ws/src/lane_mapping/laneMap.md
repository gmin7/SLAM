To run the lane mapping package, run each command in a new thread/terminal:

1. `roslaunch caffeine caffeine_gazebo.launch world:="/home/michelle/Michelle/SLAM/slam-ws/src/igvc_world/worlds/igvc_basic.world"`
2. `roslaunch caffeine caffeine_rviz.launch`
3. `roslaunch gmap gmapping_navigation.launch`
4. `roslaunch lane_mapping lane_detection.launch`
