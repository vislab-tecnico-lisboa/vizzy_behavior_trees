#!/usr/bin/env python 

#Joao Avelino, 2020.
#ISR-Lisboa / IST

#A template of a GeneralActionlib actionserver for you to use

#ROS imports
import rospy
import actionlib

#General action files
from vizzy_behavior_trees.msg import GeneralAction, GeneralFeedback, GeneralResult

#Your other imports here
#
#
#import ...

class TemplateActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = GeneralFeedback()
    _result = GeneralResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GeneralAction, execute_cb=self.execute_cb, auto_start = False)

        #Other initializations that you need
        #
        # self._somepublisher = ...
        # self._somesubscriber = ...
        # self._some_actionclient = ...
        self._as.start()
      
    def execute_cb(self, goal):

        #What to do when this receives a goal!

        #5Hz control loop for example
        dt = 1.0/5.0
        r = rospy.Rate(1.0/dt)
        success = True
        self._feedback.percentage = 0

        ##Get the action's constants here
        #constant1 = atof(goal.constants[0])
        #constant2 = goal.constants[1]
        #constant3 = atoi(goal.constants[2])
        #... whathever you need...

        ##Get actions's variables here
        #variable1 = goal.variables[0]
        #variable2 = atof(goal.variables[1])
        #... whathever you need...

        #Some code that you migh need...
        #....

        # Start executing the action
        while (not rospy.is_shutdown()) and (self._feedback.percentage < 100):

            #Check if the action was preempted!
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            
            #Code of an interaction of your action... 
            # Lots of code here....
            # -------

            #After one iteration you should update the progress if possible. It might be useful for debug
            progress = 1234123/523452341
            self._feedback.percentage = progress*100

            #Publish the feedback
            self._as.publish_feedback(self._feedback)

            r.sleep()
        

        ##Last instructions before returning!
        # ....... Code ......
        #
          
        if success:
            #Set your result as a string! :)
            self._result.result = "A very interesting result for science!"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
        else:
            self._as.set_aborted()
            #Set your result in case of failure
            self._result.result = "Nao consigo andar. Um obstaculo!"
            rospy.loginfo('%s: Failed' % self._action_name)
        
if __name__ == '__main__':
    rospy.init_node('template_action_name')
    server = TemplateActionServer(rospy.get_name())
    rospy.spin()