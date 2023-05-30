#!/usr/bin/env python3

import py_trees
# from py_trees.behaviour import Behaviour
import rospy
import time
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped

# from autonomous_task_NAK.srv import intervention_getpoint
# from std_srvs.srv import SetBool

###   Move to point behaviour
class GoToPoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, pick_or_place, location):
        super(GoToPoint, self).__init__(name)

        # attach to blackboard and allow read access to object name
        self.blackboard = self.attach_blackboard_client(name=self.name)
                
        # Blackboard variable to keep a track of which pickup location is next
        # self.blackboard.register_key(
        #     "location", access=py_trees.common.Access.READ)
        # self.blackboard.register_key(
        #     "location", access=py_trees.common.Access.WRITE)
        # self.blackboard.register_key(
        #     "pick_or_place", access=py_trees.common.Access.READ)
        # self.blackboard.register_key(
        #     "pick_or_place", access=py_trees.common.Access.WRITE)
        
        self.location = location
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

        pose = PoseStamped()
        pose.pose.position.x = self.location[0]
        pose.pose.position.y = self.location[1]
        pose.pose.position.z = self.location[2]

        self.server_set_goal(pose)
        
        print("curr pickup loc: ", self.location)


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


if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")

    go_up1 = GoToPoint("go_up1",'',[0.4,0.4,-0.30,0,0,0])
    
    go_pick = GoToPoint("go_pick",'pick',[0.4,0.4,-0.14,0,0,0])
    
    go_up2 = GoToPoint("go_up1",'',[0.4,0.4,-0.30,0,0,0])

    go_up3 = GoToPoint("go_up2",'',[0.8,0.8,-0.25,0,0,0])

    go_place = GoToPoint("go_place",'place',[0.8,0.8,-0.14,0,0,0])
    
    go_up4 = GoToPoint("go_up2",'',[0.8,0.8,-0.25,0,0,0])

    # go_up1 = GoToPoint("go_up",'',[0.8,0.4,-0.15,0,0,0])

    # go_up3 = GoToPoint("go_place",'',[0.8,0.6,-0.25,0,0,0])

    # go_place = GoToPoint("go_place",'place',[0.8,0.6,-0.14,0,0,0])
    
    # go_up4 = GoToPoint("go_place",'',[0.8,0.6,-0.25,0,0,0])
    # go_up2 = GoToPoint("go_up",'',[0.8,0.4,-0.15,0,0,0])

    # Create Behavior Tree
    root = py_trees.composites.Sequence(name="Pick n Place", memory=True)

    root.add_children([go_up1, go_pick, go_up2, go_up3, go_place, go_up4])

    # Add behaviors to the tree
    # root.add_children([go_up1, go_pick, go_up2, go_up3, go_place, go_up4])#, go_up1]) #, go_place, go_up2])

    # Display the tree before executing it
    py_trees.display.render_dot_tree(root)

    # try:
    #     print("Call setup for all tree children")
    #     root.setup_with_descendants() 
    #     print("Setup done!\n\n")
    #     py_trees.display.ascii_tree(root)
        
    #     for _ in range(20000):
    #         root.tick_once()
    #         time.sleep(1)
    # except KeyboardInterrupt:
    #     pass


 

    # Shutdown ROS
    rospy.signal_shutdown("Behavior tree finished.")
