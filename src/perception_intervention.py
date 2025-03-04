#!/usr/bin/env python3

import py_trees
# from py_trees.behaviour import Behaviour
import rospy
import copy
import time
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped

from autonomous_task_NAK.srv import intervention_getpoint
from autonomous_task_NAK.srv import MarkerPose
from std_srvs.srv import SetBool

###   Move to point behaviour
class GoToPoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, pick_or_place):
        super(GoToPoint, self).__init__(name)

        # attach to blackboard and allow read access to object name
        self.blackboard = self.attach_blackboard_client(name=self.name)
                
        # Blackboard variable to keep a track of which pickup location is next
        self.blackboard.register_key(
            "location", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "location", access=py_trees.common.Access.WRITE)
        
        self.pick_or_place = pick_or_place
        
    def setup(self):
        self.logger.debug("  %s [GoPick::setup()]" % self.name)
        # service used to set goal of move behaviour
        rospy.wait_for_service('/set_desired')

        # service used to check status of move behaviour
        rospy.wait_for_service('/goal_reached')

        # service for pump - needs to be called in terminate
        service_name = '/turtlebot/swiftpro/vacuum_gripper/set_pump'
        rospy.wait_for_service(service_name)

        try:
            self.server_set_goal = rospy.ServiceProxy(
                '/set_desired', intervention_getpoint)
            self.server_check_reached = rospy.ServiceProxy(
                '/goal_reached', Trigger)
            
            self.set_pump_service = rospy.ServiceProxy(
                service_name, SetBool)
            
            self.logger.debug(
                "  %s [GoToPoint::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [GoToPoint::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GoToPoint::initialise()]" % self.name)
        self.server_set_goal(self.blackboard.location)
        # print(self.name)
        print('normal: ', self.blackboard.location.pose.position.z)
        if self.name == 'up2':
            curr_loc = copy.deepcopy(self.blackboard.location)
            curr_loc.pose.position.x = curr_loc.pose.position.x + 0.40
            curr_loc.pose.position.y = curr_loc.pose.position.y - 0.40
            curr_loc.pose.position.z = curr_loc.pose.position.z - 0.20
            print('up pose: ', curr_loc.pose.position.z)
            self.server_set_goal(curr_loc)

        if self.name == 'place':
            curr_loc = copy.deepcopy(self.blackboard.location)
            curr_loc.pose.position.x = curr_loc.pose.position.x + 0.40
            curr_loc.pose.position.y = curr_loc.pose.position.y - 0.40
            curr_loc.pose.position.z = curr_loc.pose.position.z - 0.024
            print('pick pose: ', curr_loc.pose.position.z)
            self.server_set_goal(curr_loc)

        if self.name == 'up':
            curr_loc = copy.deepcopy(self.blackboard.location)
            curr_loc.pose.position.z = curr_loc.pose.position.z - 0.10
            print('up pose: ', curr_loc.pose.position.z)
            self.server_set_goal(curr_loc)

        if self.name == 'pick':
            curr_loc = copy.deepcopy(self.blackboard.location)
            curr_loc.pose.position.z = curr_loc.pose.position.z - 0.025
            print('pick pose: ', curr_loc.pose.position.z)
            self.server_set_goal(curr_loc)
        
        # print("curr pickup loc: ", self.location)


    def update(self):

        self.logger.debug("  {}: call service /goal_reached".format(self.name))

        resp = self.server_check_reached(TriggerRequest())

        if resp.success:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Call the service to turn on the pump (argument: True)
        if self.pick_or_place == 'pick':
            response = self.set_pump_service(True)
            if response.success:
                rospy.loginfo("Pump turned ON")
            else:
                rospy.logwarn("Failed to turn ON the pump")
            
        elif self.pick_or_place == 'place':
            response = self.set_pump_service(False)
            if response.success:
                rospy.loginfo("Pump turned OFF")
            else:
                rospy.logwarn("Failed to turn OFF the pump")
            
        
        self.logger.debug("  %s [GoToPoint::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class SeeAruco(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SeeAruco, self).__init__(name)

        # attach to blackboard and allow read access to object name
        self.blackboard = self.attach_blackboard_client(name=self.name)
                
        # Blackboard variable to keep a track of which pickup location is next
        self.blackboard.register_key(
            "location", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "location", access=py_trees.common.Access.WRITE)
        
        
    def setup(self):
        self.logger.debug("  %s [SeeAruco::setup()]" % self.name)
        # service used to set goal of move behaviour

        rospy.wait_for_service('/marker_pose')

        try:
            self.server_get_goal = rospy.ServiceProxy(
                '/marker_pose', MarkerPose)            
            self.logger.debug(
                "  %s [SeeAruco::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [SeeAruco::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SeeAruco::initialise()]" % self.name)

    def update(self):

        self.logger.debug("  {}: call service /server_get_goal".format(self.name))

        

        resp = self.server_get_goal(True).pose_msg

        curr_z = resp.pose.position.z
        print('z pos: ', curr_z)

        if curr_z == 0:
            return py_trees.common.Status.RUNNING
        else:
            # print('before: ', resp.pose.position.y)
            # resp.pose.position.y += 10
            self.blackboard.location = resp
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # Call the service to turn on the pump (argument: True)
        self.logger.debug("  %s [SeeAruco::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")
    # print("nonooooo")

    see_aruco = SeeAruco('see_aruco')
    # go_pick_up = GoToPoint("up",'')
    go_up1 = GoToPoint("up",'up')
    pick = GoToPoint("pick",'pick')
    go_up2 = GoToPoint("up",'up')

    go_up3 = GoToPoint("up2",'up2')
    place = GoToPoint("place",'place')
    go_up4 = GoToPoint("up2",'up2')



    # Create Behavior Tree
    root = py_trees.composites.Sequence(name="Mobile Pick n Place", memory=True)

    root.add_children([see_aruco, go_up1, pick, go_up2, go_up3, place, go_up4])

    # Add behaviors to the tree
    # root.add_children([go_pick, go_up1, go_place, go_up2])

    # Display the tree before executing it
    py_trees.display.render_dot_tree(root)

    try:
        print("Call setup for all tree children")
        root.setup_with_descendants() 
        print("Setup done!\n\n")
        py_trees.display.ascii_tree(root)
        
        for _ in range(20000):
            root.tick_once()
            time.sleep(1)
    except KeyboardInterrupt:
        pass


 

    # Shutdown ROS
    rospy.signal_shutdown("Behavior tree finished.")
