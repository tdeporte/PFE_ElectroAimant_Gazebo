#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandBool, SetMode


class DroneController:
    
    def __init__(self):
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
    #Armement
    def set_arm(self):
        self.arm_service(True)
        rospy.loginfo("Armed")
        
    #Désarmement 
    def set_disarm(self):
        self.arm_service(False)
        rospy.loginfo("Disarmed")

    #Décollage    
    def set_offboard(self):
        self.mode_service(base_mode = 0 , custom_mode="OFFBOARD")
        rospy.loginfo("Mode set to Offboard")
        
    #Passage en mode "Atterissage automatique" 
    def set_auto_land(self):
        self.mode_service(base_mode = 0 , custom_mode="AUTO.LAND")
        rospy.loginfo("Mode set to Autoland")
    