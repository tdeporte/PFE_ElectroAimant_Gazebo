#!/usr/bin/env python3

import unittest
from DroneController import DroneController
import rospy
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from mavros_msgs.msg import State
import time
import math
import sys

PKG = 'px4'

class DroneTests(unittest.TestCase):
    def __init__(self, *args):
        super(DroneTests, self).__init__(*args)
        

    def setUp(self):
        super(DroneTests, self).setUp()
                
        self.controller = DroneController()
        
        self.timeout = 1
        self.changed_mode = {"AUTO.LOITER" : False , "AUTO.LAND" : False}
        self.topics_ready = {'local_pos' : False , 'camera' : False , 'state' : False}
        self.tests_ready = False
        
        if( len(sys.argv) != 4):
            self.position_test = (2 , 2 , 1)
        else:
            self.position_test = ( float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]) )
        
        self.rate = rospy.Rate(25.0)
        self.pos = PoseStamped()
        self.local_position = PoseStamped()
        self.camera = Image()
        self.state = State()
        
        self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose' , PoseStamped , self.local_pos_callback)
        self.camera_sub = rospy.Subscriber('/iris/camera_red_iris/image_raw' , Image, self.camera_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local' , PoseStamped , queue_size=10)
        
        time.sleep(5)

    #
    # Callback functions
    #
    
    def state_callback(self , data):
        self.state = data
        
        if not self.topics_ready['state']:
            self.topics_ready['state'] = True
    
    def local_pos_callback(self, data):
        self.local_position = data
    
        if not self.topics_ready['local_pos']:
            self.topics_ready['local_pos'] = True
            
    def camera_callback(self , data):
        self.camera = data
        
        if not self.topics_ready['camera']:
            self.topics_ready['camera'] = True
    
    #
    # Helper methods
    #
    
    def set_offboard(self):
        for j in range(100):
            self.pos.pose.position.x = 0.0
            self.pos.pose.position.y = 0.0
            self.pos.pose.position.z = 0.0
            
            self.pos_pub.publish(self.pos)
            self.rate.sleep()
                        
    def move_to(self , x , y , z):
        for j in range(500):
            self.pos.pose.position.x = x
            self.pos.pose.position.y = y
            self.pos.pose.position.z = z
            
            self.pos_pub.publish(self.pos)
            self.rate.sleep()
            
    #
    # Tests methods
    #
    
    def test_subscribers(self):   
        rospy.loginfo("Subscribers test")     
        time.sleep(self.timeout)

        for value in self.topics_ready.values():
            if(value == False):
                self.tests_ready = False
                break
            else:
                self.tests_ready = True
                
        self.assertTrue(self.tests_ready , 
        ("Subscriber to all topics failed | topics flags: {0}".
            format(self.topics_ready)))
        
    
    def test_arming(self):
        rospy.loginfo("Arming test")
        self.controller.set_arm()
        time.sleep(1)
        
        self.assertTrue(self.state.armed , 
        ("Arming Failed | Arming : {0}".
            format(self.state.armed)))
        
    def test_disarming(self):
        rospy.loginfo("Disarming test")
        self.controller.set_disarm()
        time.sleep(1)
        
        self.assertTrue(not self.state.armed , 
        ("Arming Failed | Arming : {0}".
            format(self.state.armed)))
        
        
    def test_position(self):
        rospy.loginfo("Position test")
        good_position = False
        
        self.controller.set_arm()
        self.set_offboard()
        self.mode_service(base_mode = 0 , custom_mode="OFFBOARD")
        self.move_to(self.position_test[0] , self.position_test[1] , self.position_test[2])
        
        x = self.position_test[0] - self.local_position.pose.position.x
        y = self.position_test[1] - self.local_position.pose.position.y
        z = self.position_test[2] - self.local_position.pose.position.z
        
        distance = math.sqrt( (x**2) + (y**2) + (z**2) )

        if(distance < 0.5):
            good_position = True
            
        self.assertTrue(good_position , 
        ("Move to position failed | Position distance: {0}".
            format(distance)))
        
    
    def test_modes(self):
        rospy.loginfo("Modes test")
        for key in self.changed_mode.keys():
            if(key == "OFFBOARD"):
                self.controller.set_arm()
                time.sleep(1)
                self.set_offboard()
            
            self.mode_service(base_mode = 0 , custom_mode=str(key))
            
            time.sleep(1)
            if( str(self.state.mode) == str(key) ):
                self.changed_mode[key] = True
            
            time.sleep(1)
            
        changed_mode_work = True
        for value in self.changed_mode.values():
            if(value == False):
                changed_mode_work = False
                
        self.assertTrue(changed_mode_work , 
        ("Test changed all mode failed | Modes flags: {0}".
            format(self.changed_mode)))

if __name__ == '__main__':

    import rostest
    
    rospy.init_node("DroneTest" , anonymous=True)
    rostest.rosrun(PKG, 'mavros_drone_tests', DroneTests)