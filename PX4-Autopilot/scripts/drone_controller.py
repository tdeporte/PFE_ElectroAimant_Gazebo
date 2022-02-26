#!/usr/bin/env python3

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode


class DroneController:
    
    def __init__(self):
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
    def setArm(self):
        self.arm_service(True)
        rospy.loginfo("ARM ACTIVATED")
        
    def setDisarm(self):
        self.arm_service(False)
        rospy.loginfo("ARM DISACTIVATED")
        
    def setOffboard(self):
        self.mode_service(base_mode = 0 , custom_mode="OFFBOARD")
        rospy.loginfo("OFFBOARD MODE SET")
        
    def setAutoLand(self):
        self.mode_service(base_mode = 0 , custom_mode="AUTO.LAND")
        rospy.loginfo("AUTOLAND MODE SET")
        