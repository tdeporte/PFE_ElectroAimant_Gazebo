#!/usr/bin/env python3

from numpy import float64
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from DroneController import DroneController
import threading
import time
from DroneCamera import DroneCamera

from gazebo_msgs.msg import ODEPhysics
from geometry_msgs.msg import Vector3
from gazebo_msgs.srv import SetPhysicsProperties
from std_msgs.msg import Float64

class Drone:
    
    def __init__(self):
        rospy.init_node('drone' , anonymous=True)
        
        self.rate = rospy.Rate(25.0)
        self.controller = DroneController()
        self.camera = DroneCamera()
        
        self.gravity_service = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local' , PoseStamped , queue_size=10)
        self.pos = PoseStamped()

        self.laser_distance = 0.0    
        self.thread_on = True
        
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
        while not rospy.is_shutdown() and (self.thread_on == True):
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
        
    def laser_callback(self,msg):
        self.laser_distance = msg.range
        rospy.loginfo("DISTANCE FROM PLATE : %f", self.laser_distance)


if __name__ == '__main__':
    try:
        drone = Drone()
        drone.controller.setArm()
        drone.setOffboard()
                        
        thread = threading.Thread(target= drone.standyTo)
        thread.start()
        
        drone.camera.start_stream()
        time.sleep(5)
        drone.setTargetPosition(0, 0 , 1)
    

        height, width, channels = drone.camera.cv_image.shape

        tolerance = 30
        low_width_threshold = width/2 - tolerance
        high_width_threshold = width/2 + tolerance
        low_height_threshold = height/2 - tolerance
        high_height_threshold = height/2 + tolerance

        step =  0.2

        while(1):
            time.sleep(0.5)

            sub = rospy.Subscriber('/iris_odom/range_down', Range, drone.laser_callback)

            center = drone.camera.get_center_QR_code()
            
            if(center[0]!=None):
                if(center[0]<low_width_threshold or center[0]>high_width_threshold
                or center[1]<low_height_threshold or center[1]>high_height_threshold):
                    if(center[0]<width/2):
                        drone.setTargetPosition(drone.pos.pose.position.x , drone.pos.pose.position.y + step, drone.pos.pose.position.z)
                    else:
                        drone.setTargetPosition(drone.pos.pose.position.x , drone.pos.pose.position.y - step, drone.pos.pose.position.z)
                    if(center[1]<height/2):
                        drone.setTargetPosition(drone.pos.pose.position.x - step , drone.pos.pose.position.y, drone.pos.pose.position.z)
                    else:
                        drone.setTargetPosition(drone.pos.pose.position.x + step , drone.pos.pose.position.y, drone.pos.pose.position.z)
                else:
                    break

        drone.controller.setAutoLand()
        drone.controller.setDisarm()
        
        time.sleep(3)
        drone.camera.stop_stream()
        rospy.spin()


        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interruption !")