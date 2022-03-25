#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2


class DroneCamera:
  
    def __init__(self):      
      self.bridge = CvBridge() #Objet permettant la conversion entre format de la camera et opencv
      self.cv_image = 0 #Image courante de la caméra
      self.camera_sub = rospy.Subscriber('/iris/camera_red_iris/image_raw' , Image, self.image_callback)
      
      rospy.loginfo("Camera started")
      
    #Fonction callback de récupération et conversion du retour camera
    def image_callback(self, img):
      self.bridge = CvBridge()
      self.cv_image = self.bridge.imgmsg_to_cv2(img)
      
      if(self.stream_camera == True):
        cv2.imshow("CameraWindow", cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2RGB))
        cv2.waitKey(1)
      else:
        cv2.destroyAllWindows()
        cv2.waitKey(1)
         
    def get_center_QR_code(self):
      qrDecoder = cv2.QRCodeDetector()
      detected,bbox = qrDecoder.detect(self.cv_image)
      if detected:
        center = int((bbox[0][0][0] + bbox[0][2][0])/2) , int((bbox[0][0][1] + bbox[0][2][1])/2)
        return center
      else:
          return None,None
        
    def get_image(self):
      self.current_image = self.cv_image
      rospy.loginfo("Get camera image")
      
    def save_image(self, filename):
      self.current_image = self.cv_image
      cv2.imwrite(filename , self.current_image)
      rospy.loginfo("Save camera image")  
    
    def start_stream(self):
      self.stream_camera = True
      rospy.loginfo("Stream started")
      
    def stop_stream(self):
      self.stream_camera = False
      rospy.loginfo("Stream stopped")
    

    