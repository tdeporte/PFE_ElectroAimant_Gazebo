#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from DroneController import DroneController
import threading
import time
from DroneCamera import DroneCamera

class Drone:
    
    def __init__(self):
        rospy.init_node('drone' , anonymous=True)
        
        self.rate = rospy.Rate(25.0)
        self.controller = DroneController()
        self.camera = DroneCamera()
        
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
            
    def standyTo(self):
        while not rospy.is_shutdown():
            self.pos_pub.publish(self.pos)
            self.rate.sleep()
            
    def setTargetPosition(self, x , y , z):
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
            
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
                        
        thread = threading.Thread(target= drone.standyTo)
        thread.start()
        
        drone.camera.start_stream()
        time.sleep(5)
        drone.setTargetPosition(0 , 0 , 2.0)
        
        #time.sleep(5)
        #drone.camera.stop_stream()

        height, width, channels = drone.camera.cv_image.shape

        tolerance = 50
        low_width_threshold = width/2 - tolerance
        high_width_threshold = width/2 + tolerance
        low_height_threshold = height/2 - tolerance
        high_height_threshold = height/2 + tolerance

        step =  0.2
        while(1):
            time.sleep(0.5)
            center = drone.camera.get_center_QR_code()
            if(center[0]!=None):
                rospy.loginfo("CENTER (%d,%d)",center[0],center[1])
                if(center[0]<low_width_threshold or center[0]>high_width_threshold
                or center[1]<low_height_threshold or center[1]>high_height_threshold):
                    if(center[0]<width/2):
                        rospy.loginfo("GO LEFT")
                        drone.moveTo(drone.pos.pose.position.x , drone.pos.pose.position.y + step, drone.pos.pose.position.z)
                    else:
                        rospy.loginfo("GO RIGHT")
                        drone.moveTo(drone.pos.pose.position.x , drone.pos.pose.position.y - step, drone.pos.pose.position.z)
                    if(center[1]<height/2):
                        rospy.loginfo("GO BACK")
                        drone.moveTo(drone.pos.pose.position.x - step , drone.pos.pose.position.y, drone.pos.pose.position.z)
                    else:
                        rospy.loginfo("GO FRONT")
                        drone.moveTo(drone.pos.pose.position.x + step , drone.pos.pose.position.y, drone.pos.pose.position.z)
                else:
                    rospy.loginfo("CORRECT POSITION")
                    break



        #rospy.spin()

        
        #drone.controller.setAutoLand()
        #drone.controller.setDisarm()


        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interruption !")