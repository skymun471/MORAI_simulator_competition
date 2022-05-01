    #!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:

    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)
        
        lower_white = np.uint8([10, 0, 200])
        upper_white = np.uint8([30, 50, 255])

        #white up : 0,0,200 / 40,50,255

        lower_yellow = np.uint8([20, 100, 100])
        upper_yellow = np.uint8([30, 255, 255])
        
        white_mask = cv2.inRange(img_hsv, lower_white, upper_white)
        yellow_mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        mask_p = cv2.bitwise_or(white_mask, yellow_mask)
        mask_1 = cv2.bitwise_and(self.img_bgr, self.img_bgr, mask = mask_p)
        
        #orange : 20, 100, 100 / 30, 255, 255
        #white : 0,0,220 / 180,30,255
        #white : 0, 0, 130 /40, 30, 255
    
        # mask_1 = cv2.cvtColor(mask_1, cv2.COLOR_GRAY2BGR)

        img_concat = np.concatenate([self.img_bgr, img_hsv, mask_1], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 

if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 