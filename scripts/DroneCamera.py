#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import numpy as np
import time


class DroneCamera:
  
    def __init__(self):      
      self.bridge = CvBridge()
      self.cv_image = 0
      self.camera_sub = rospy.Subscriber('/iris/camera_red_iris/image_raw' , Image, self.image_callback)
      self.stream_camera = False
      self.current_image = 0
      
      rospy.loginfo("CAMERA START")
      
      
    def image_callback(self, img):
      self.bridge = CvBridge()
      self.cv_image = self.bridge.imgmsg_to_cv2(img)
      if(self.stream_camera == True):
        cv2.imshow("CameraWindow", cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2RGB))
        cv2.waitKey(1)
      else:
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        
        
    
    def get_image(self):
      self.current_image = self.cv_image
      rospy.loginfo("GET CAMERA IMAGE")
      
    def save_image(self, filename):
      self.current_image = self.cv_image
      cv2.imwrite(filename , self.current_image)
      rospy.loginfo("SAVE CAMERA IMAGE")  
    
    def start_stream(self):
      self.stream_camera = True
      rospy.loginfo("START STREAM")
      
    def stop_stream(self):
      self.stream_camera = False
      rospy.loginfo("STOP STREAM")

    def get_center_QR_code(self):
      rospy.loginfo("ENTER QR CODE DETECTOR")
      qrDecoder = cv2.QRCodeDetector()
      detected,bbox = qrDecoder.detect(self.cv_image)
      if detected:
        center = int((bbox[0][0][0] + bbox[0][2][0])/2) , int((bbox[0][0][1] + bbox[0][2][1])/2)
        rospy.loginfo("QR code center : (%d,%d)", center[0],center[1])
        return center
      else:
          rospy.loginfo("QR Code not detected")
          return None,None

    