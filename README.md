# Hands-on Intervention

last update:
5May 9.04: Package made, orientation of EE set

## HOW TO RUN:

**intervention perception**
- roslaunch turtlebot_simulation turtlebot_hoi.launch
- rosrun hands-on-intervention FK_diff_drive.py
- rosrun pose-graph-slam integration.py 
- rosrun hands_on_perception aruco_pose_realsense_stonefish.py
- rosrun hands-on-intervention intervention_integrated_stonefish.py 
- rosrun autonomous_task_NAK perception_intervention


**intervention perception planning**
- roslaunch frontier_explorationb turtlebot_circuit1.launch
- rosrun frontier_explorationb frontier_exploration_integrated.py
- rosrun frontier_explorationb move_to_pt.py
- rosrun hands_on_perception aruco_pose_realsense_stonefish.py
- rosrun hands-on-intervention intervention_integrated_stonefish.py 
- rosrun autonomous_task_NAK perception_intervention_planning