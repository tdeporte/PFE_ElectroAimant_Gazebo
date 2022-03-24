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

class Drone:
    
    def __init__(self):
        rospy.init_node('drone' , anonymous=True)
        
        self.rate = rospy.Rate(10.0)
        self.controller = DroneController() #Objet controller du drone
        self.camera = DroneCamera() #Objet camera du drone
        self.thread_on = True #Si True envoie une position au drone sinon non
        self.phase = 1 #Phase du drone (docking = 1 / undocking = 0)
        self.center_pos = PoseStamped() #Position du centre du qr code
        self.gap = 0.25 #Ecart maximum entre la plaque et le drone pour le docking
        
        #Mettre en pause la simulation ou la relancer
        self.pause_sim = rospy.ServiceProxy('/gazebo/pause_physics' , Empty)
        self.unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics' , Empty)
        
        #Modifier la position cible du drone
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local' , PoseStamped , queue_size=10)
        self.pos = PoseStamped()
        
        #Récupération de la distance renvoyée par le laser dans drone.laser_distance 
        self.laser_sub = rospy.Subscriber('/iris_odom/range_down', Range, self.laser_callback)
        self.laser_distance = 0.0  #Distance entre le laser et un objet
        
        #Récupérer la position courante du drone
        self.current_pos_sub = rospy.Subscriber('/mavros/local_position/pose' , PoseStamped , self.current_pos_callback)
        self.current_pos = PoseStamped()
        
        #Si des arguments de tolerance et step sont donnée sinon on donne des valeurs par défaut
        if( len(sys.argv) >= 3 ):
            #Fait varier les dimensions de la cible au centre de l'image
            self.tolerance = int(sys.argv[1])
            
            #Distance parcourue par le drone lors du replacement 
            self.step = float(sys.argv[2])
        else:
            self.tolerance = 30
            self.step = 0.2
            
    #Fonction callback du topic de position courante du drone
    def current_pos_callback(self, data):
        self.current_pos = data
        
    #Fonction de déplacement vers une position avec un standby dans un thread
    def standby_to(self):
        while not rospy.is_shutdown():
            if( self.thread_on == True ):
                self.pos_pub.publish(self.pos)
                self.rate.sleep()
    
    #Modifie la position cible du drone
    def set_target_position(self, x , y , z):
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        
    #Fonction callback du topic de distance
    def laser_callback(self,msg):
        self.laser_distance = msg.range
    
    #Récupérer les touches pressées
    def on_press(self ,key):
        try:
            if(key.char == "d"):
                self.thread_on = True
                self.controller.setArm()
                self.controller.setOffboard()
                rospy.loginfo("Docking ...")
                self.phase = 1
                
                #On fait monter le drone d'1 mètre
                drone.set_target_position(self.pos.pose.position.x, self.pos.pose.position.y , self.current_pos.pose.position.z + 1 )
            elif(key.char == "u"):
                rospy.loginfo("Undocking ...")
                self.phase = 0

        except AttributeError:
            print('special key pressed: {0}'.format(key))
    
    #Fonction d'un threadà part pour écouter les touches
    def listen_key(self):
        with keyboard.Listener(on_press=drone.on_press) as listener:
            listener.join()
            


if __name__ == '__main__':
    try:
        drone = Drone()
                        
        thread_standby = threading.Thread(target= drone.standby_to)
        thread_standby.start()
        
        thread_key = threading.Thread(target= drone.listen_key)
        thread_key.start()
        
        time.sleep(2)
        
        rospy.loginfo("Keys listen ready")
        
        # drone.set_target_position(0, 0 , 1)
    
        #Dimensions de l'image
        height, width, channels = drone.camera.cv_image.shape

        #Dimensions de la cible au centre de l'image
        low_width_threshold = width/2 - drone.tolerance
        high_width_threshold = width/2 + drone.tolerance
        low_height_threshold = height/2 - drone.tolerance
        high_height_threshold = height/2 + drone.tolerance

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
                        drone.set_target_position(drone.pos.pose.position.x , drone.pos.pose.position.y + drone.step, drone.pos.pose.position.z)
                    #Si le centre du QR code est à droite du centre de l'image    
                    else:
                        #On se déplace sur l'axe y
                        drone.set_target_position(drone.pos.pose.position.x , drone.pos.pose.position.y - drone.step, drone.pos.pose.position.z)
                    #Si le centre du QR code est en bas du centre de l'image    
                    if(center[1]<height/2):
                        #On se déplace sur l'axe x
                        drone.set_target_position(drone.pos.pose.position.x - drone.step , drone.pos.pose.position.y, drone.pos.pose.position.z)
                    #Si le centre du QR code est au dessus du centre de l'image 
                    else:
                        #On se déplace sur l'axe x
                        drone.set_target_position(drone.pos.pose.position.x + drone.step , drone.pos.pose.position.y, drone.pos.pose.position.z)
                #Si le centre du QR code est au centre de l'image
                else:
                    drone.center_pos.pose.position.x = drone.current_pos.pose.position.x
                    drone.center_pos.pose.position.y = drone.current_pos.pose.position.y
                    drone.center_pos.pose.position.z = drone.current_pos.pose.position.z
                    rospy.loginfo("Center QR Code find")
                    break

        time.sleep(0.5)
                
        while(1):
                
            if(drone.phase == 1):
                #Tant que le laser renvoit  une distance avec la plaque supérieure à 0.25
                if(drone.laser_distance > drone.gap):
                    time.sleep(0.5)
                    drone.set_target_position(drone.center_pos.pose.position.x , drone.center_pos.pose.position.y, drone.current_pos.pose.position.z + (drone.step *2) )
                else:
                    drone.thread_on = False
                    drone.pause_sim(EmptyRequest())
            elif(drone.phase == 0):
                time.sleep(2)
                drone.unpause_sim( EmptyRequest())
                drone.phase = -1
        
        drone.controller.setAutoLand()
        drone.controller.setDisarm()          
        rospy.spin()
          

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interruption !")