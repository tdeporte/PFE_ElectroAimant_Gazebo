#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2


class DroneCamera:
  
    def __init__(self, tolerance):   
      self.tolerance = tolerance   
      self.bridge = CvBridge() #Objet permettant la conversion entre format de la camera et opencv
      self.cv_image = 0 #Image courante de la caméra
      self.camera_sub = rospy.Subscriber('/iris/camera_red_iris/image_raw' , Image, self.image_callback)
      
      rospy.loginfo("Camera started")
      
    #Fonction callback de récupération et conversion du retour camera
    def image_callback(self, img):
      self.bridge = CvBridge()
      self.cv_image = self.bridge.imgmsg_to_cv2(img)
      center = self.get_center_QR_code()
      if(any(map(lambda elem: elem is not None, center))):
        self.cv_image = cv2.circle(self.cv_image, center, radius=0, color=(255, 0, 0), thickness=10)

      height, width, channels = self.cv_image.shape
      start_point = (width/2 - self.tolerance,height/2 - self.tolerance)
      end_point = (width/2 + self.tolerance ,height/2 + self.tolerance  )
      self.cv_image = cv2.rectangle(self.cv_image,(int(start_point[0]), int(start_point[1])), (int(end_point[0]), int(end_point[1])), (0, 255, 0), 2)
      cv2.imshow("CameraWindow", cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2RGB))
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
    

    