To run the lane mapping package, run each command in a new thread/terminal:

1. `roslaunch husky_gazebo husky_playpen.launch`
2. `roslaunch husky_viz view_robot.launch`
3. `roslaunch gmap gmapping_navigation.launch`
4. `roslaunch lane_mapping lane_detection.launch`
