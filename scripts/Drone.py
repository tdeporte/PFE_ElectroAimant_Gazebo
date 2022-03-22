#!/usr/bin/env python3

from numpy import float64
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from DroneController import DroneController
import threading
import time
from DroneCamera import DroneCamera
import sys, getopt


class Drone:
    
    def __init__(self):
        rospy.init_node('drone' , anonymous=True)
        
        self.rate = rospy.Rate(25.0)
        self.controller = DroneController()
        self.camera = DroneCamera()
        
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local' , PoseStamped , queue_size=10)
        self.pos = PoseStamped()

        self.laser_distance = 0.0    
        
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
        
    def laser_callback(self,msg):
        self.laser_distance = msg.range
        #rospy.loginfo("DISTANCE FROM PLATE : %f", self.laser_distance)
            

if __name__ == '__main__':
    try:
        drone = Drone()

        drone.controller.setArm()
        drone.setOffboard()
                        
        thread = threading.Thread(target= drone.standyTo)
        thread.start()
        
        drone.camera.start_stream()
        time.sleep(5)
        drone.setTargetPosition(0 , 0 , 0.5)
        
        #time.sleep(5)
        #drone.camera.stop_stream()

        #Dimensions de l'image
        height, width, channels = drone.camera.cv_image.shape

        #Fait varier les dimensions de la cible au centre de l'image
        tolerance = int(sys.argv[1])

        #Dimensions de la cible au centre de l'image
        low_width_threshold = width/2 - tolerance
        high_width_threshold = width/2 + tolerance
        low_height_threshold = height/2 - tolerance
        high_height_threshold = height/2 + tolerance

        #Distance parcourue par le drone lors du replacement 
        step =  float(sys.argv[2])

        #Boucle de replacement du drone en dessous du centre du QR code
        while(1):
            #Stabilisation du drone 
            time.sleep(1)

            #Récupération de la distance renvoyée par le laser dans drone.laser_distance 
            sub = rospy.Subscriber('/iris_odom/range_down', Range, drone.laser_callback)

            #Récupération des coordonées du centre du QR code dans l'image
            center = drone.camera.get_center_QR_code()
            
            #Si on détecte un QR code
            if(any(map(lambda elem: elem is not None, center))):

                #rospy.loginfo("CENTER (%d,%d)",center[0],center[1])

                #Si le centre du QR code est dans la cible 
                if(center[0]<low_width_threshold or center[0]>high_width_threshold
                or center[1]<low_height_threshold or center[1]>high_height_threshold):
                    #Si le centre du QR code est à gauche du centre de l'image
                    if(center[0]<width/2):
                        #On se déplace sur l'axe y
                        drone.setTargetPosition(drone.pos.pose.position.x , drone.pos.pose.position.y + step, drone.pos.pose.position.z)
                    #Si le centre du QR code est à droite du centre de l'image    
                    else:
                        #On se déplace sur l'axe y
                        drone.setTargetPosition(drone.pos.pose.position.x , drone.pos.pose.position.y - step, drone.pos.pose.position.z)
                    #Si le centre du QR code est en bas du centre de l'image    
                    if(center[1]<height/2):
                        #On se déplace sur l'axe x
                        drone.setTargetPosition(drone.pos.pose.position.x - step , drone.pos.pose.position.y, drone.pos.pose.position.z)
                    #Si le centre du QR code est au dessus du centre de l'image 
                    else:
                        #On se déplace sur l'axe x
                        drone.setTargetPosition(drone.pos.pose.position.x + step , drone.pos.pose.position.y, drone.pos.pose.position.z)
                #Si le centre du QR code est au centre de l'image
                else:
                    #Tant que le laser renvoit  une distance avec la plaque supérieure à 0.1
                    while(drone.laser_distance<0.1):
                        time.sleep(0.5)
                        #On fait monter le drone
                        drone.setTargetPosition(drone.pos.pose.position.x , drone.pos.pose.position.y, drone.pos.pose.position.z + step)
                    break
        rospy.spin()
        drone.controller.setAutoLand()
        drone.controller.setDisarm()    

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interruption !")