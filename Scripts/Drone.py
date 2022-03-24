#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from DroneController import DroneController
import threading
import time
from DroneCamera import DroneCamera
import sys
from std_srvs.srv import Empty, EmptyRequest  
from pynput import keyboard
from mavros_msgs.msg import State

class Drone:
    
    def __init__(self):
        rospy.init_node('drone' , anonymous=True)
        
        self.rate = rospy.Rate(10.0)
        self.controller = DroneController()
        self.camera = DroneCamera()
        
        self.pause_sim = rospy.ServiceProxy('/gazebo/pause_physics' , Empty)
        self.unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics' , Empty)
        
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local' , PoseStamped , queue_size=10)
        self.pos = PoseStamped()
        
        self.current_pos_sub = rospy.Subscriber('/mavros/local_position/pose' , PoseStamped , self.current_pos_callback)
        self.current_pos = PoseStamped()
        
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.state = State()

        self.laser_distance = 0.0    
        self.thread_on = True
        self.phase = 1
        
        self.center_pos = PoseStamped()
        
    def moveTo(self , x , y , z):
        for j in range(100):
            self.pos.pose.position.x = x
            self.pos.pose.position.y = y
            self.pos.pose.position.z = z
            
            self.pos_pub.publish(self.pos)
            self.rate.sleep()
        
        rospy.loginfo("MOVE TO : " + str(x) + " " +  str(y) + " " + str(z) )
            
    def current_pos_callback(self, data):
        self.current_pos = data
        
    def state_callback(self, data):
        self.state = data
    
    def moveToWayPoints(self, list):
        for pos in list:
            self.moveTo(pos[0] , pos[1], pos[2])
            
    def standbyTo(self):
        while not rospy.is_shutdown():
            if( self.thread_on == True ):
                # rospy.loginfo("standby")
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
        
    def laserCallback(self,msg):
        self.laser_distance = msg.range
    
    def on_press(self ,key):
        try:
            if(key.char == "l"):
                self.thread_on = True
                self.controller.setArm()
                # self.setOffboard()
                rospy.loginfo("Launching ...")
                self.phase = 1
                drone.setTargetPosition(self.pos.pose.position.x, self.pos.pose.position.y , self.current_pos.pose.position.z + 1 )
            elif(key.char == "p"):
                self.phase = 0

        except AttributeError:
            print('special key pressed: {0}'.format(key))
            
    def listenKey(self):
        with keyboard.Listener(on_press=drone.on_press) as listener:
            listener.join()
            


if __name__ == '__main__':
    try:
        drone = Drone()
    
        # drone.controller.setArm()
        # drone.setOffboard()
        
        #Récupération de la distance renvoyée par le laser dans drone.laser_distance 
        sub = rospy.Subscriber('/iris_odom/range_down', Range, drone.laserCallback)
                        
        thread_standby = threading.Thread(target= drone.standbyTo)
        thread_standby.start()
        
        thread_key = threading.Thread(target= drone.listenKey)
        thread_key.start()
        
        time.sleep(2)
        
        rospy.loginfo("LISTEN KEY READY")
        
        # drone.setTargetPosition(0, 0 , 1)
    
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

            #Récupération des coordonées du centre du QR code dans l'image
            center = drone.camera.get_center_QR_code()
            
            #Si on détecte un QR code
            if(any(map(lambda elem: elem is not None, center))):

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
                    drone.center_pos.pose.position.x = drone.current_pos.pose.position.x
                    drone.center_pos.pose.position.y = drone.current_pos.pose.position.y
                    drone.center_pos.pose.position.z = drone.current_pos.pose.position.z
                    rospy.loginfo("QR Code find")
                    break

        time.sleep(0.5)
                
        while(1):
                
            if(drone.phase == 1):
                #Tant que le laser renvoit  une distance avec la plaque supérieure à 0.1
                if(drone.laser_distance > 0.15):
                    time.sleep(0.5)
                    drone.setTargetPosition(drone.center_pos.pose.position.x , drone.center_pos.pose.position.y, drone.current_pos.pose.position.z + step + 0.1 )
                else:
                    drone.thread_on = False
                    drone.pause_sim(EmptyRequest())
            elif(drone.phase == 0):
                time.sleep(2)
                rospy.loginfo("Landing ...")
                drone.unpause_sim( EmptyRequest())
                drone.phase = -1
                
        
        
        drone.controller.setAutoLand()
        drone.controller.setDisarm()          
        rospy.spin()
          

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interruption !")