# Hands-on Intervention

last update:
5May 9.04: Package made, orientation of EE set

## HOW TO RUN:

standalone intervention hardware

- rosrun pose-graph-slam integration_hardware.py
- rosrun frontier_explorationb vw_to_wheel_frontier.py
- rosrun hands-on-intervention standalone_controller_hardware.py
- rosrun autonomous_task_NAK only_intervention.py

**intervention perception**
- roslaunch turtlebot_simulation turtlebot_hoi.launch
- rosrun hands-on-intervention FK_diff_drive.py
- rosrun pose-graph-slam integration.py 
- rosrun hands_on_perception aruco_pose_realsense_stonefish.py
- rosrun hands-on-intervention intervention_integrated_stonefish.py 
- rosrun autonomous_task_NAK perception_intervention


**intervention perception hardware**
- roslaunch turtlebot_simulation turtlebot_hoi.launch
- rosrun pose-graph-slam integration.py 
- rosrun hands-on-intervention FK_diff_drive.py
- rosrun hands_on_perception aruco_pose_realsense_hardware.py
- rosrun hands-on-intervention intervention_integrated_hardware.py 
- rosrun autonomous_task_NAK perception_intervention



**intervention perception planning**
- roslaunch frontier_explorationb turtlebot_circuit1.launch
- rosrun frontier_explorationb frontier_exploration_integrated.py
- rosrun frontier_explorationb move_to_pt.py
- rosrun hands_on_perception aruco_pose_realsense_stonefish.py
- rosrun hands-on-intervention intervention_integrated_stonefish.py 
- rosrun frontier_explorationb move_to_point_planner_integration.py 
- rosrun autonomous_task_NAK perception_intervention_planning.py