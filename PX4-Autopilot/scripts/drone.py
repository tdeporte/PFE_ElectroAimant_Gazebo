#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from drone_controller import DroneController

class Drone:
    
    def __init__(self):
        rospy.init_node('drone' , anonymous=True)
        
        self.rate = rospy.Rate(25.0)
        self.controller = DroneController()
        
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local' , PoseStamped , queue_size=10)
        self.pos = PoseStamped()
        
            
    def moveTo(self , x , y , z):
        for j in range(100):
            self.pos.pose.position.x = x
            self.pos.pose.position.y = y
            self.pos.pose.position.z = z
            
            self.pos_pub.publish(self.pos)
            self.rate.sleep()
        
        rospy.loginfo("MOVE TO : " + str(x) + " " +  str(y) + " " + str(z) )
            
    def moveToWayPoints(self, list):
        for pos in list:
            self.moveTo(pos[0] , pos[1], pos[2])
            
    def setOffboard(self):
        for j in range(100):
            self.pos.pose.position.x = 0.0
            self.pos.pose.position.y = 0.0
            self.pos.pose.position.z = 0.0
            
            self.pos_pub.publish(self.pos)
            self.rate.sleep()
            
        rospy.loginfo("OFFBOARD PREPARATION")
        
        self.controller.setOffboard()
        
        

if __name__ == '__main__':
    try:
        drone = Drone()
        
        drone.controller.setArm()
        drone.setOffboard()
        
        positions = ( (3.0 ,1.0 ,2.0), (-1.0,-3.0,2.0), (0.0, 0.0, 2.0))
        
        drone.moveToWayPoints(positions)
        
        drone.controller.setAutoLand()
        drone.controller.setDisarm()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interruption !")