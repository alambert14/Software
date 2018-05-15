#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class duckie_detector_node():

    def __init__(self):
        self.detect_pub = rospy.Publisher('/cameraduckie/detected_duckie', Image, queue_size=10)
        self.bridge = CvBridge()

        self.cam_sub = rospy.Subscriber('/camera/image_color/compressed', CompressedImage, self.callback)

        #SimpleBlobDetector Params
        params = cv2.SimpleBlobDetector_Params()

        params.minThreshold = 0
        params.maxThreshold = 256

        params.filterByArea = True
        params.minArea = 100
        
        params.filterByCircularity = True
        params.minCircularity = 0.1

        params.filterByConvexity = False
        
        params.filterByInertia = False

        self.detector = cv2.SimpleBlobDetector_create(params)
        self.red_boundary_lower = np.array([0,0,80], dtype="uint8")
        self.red_boundary_upper = np.array([50,50,200], dtype="uint8")
        

    def callback(self,img):
        print("Callback!")
        raw = self.bridge.compressed_imgmsg_to_cv2(img)
        #keypoints = self.detector.detect(raw)
        mask = cv2.inRange(raw,self.red_boundary_lower,self.red_boundary_upper)
        reverse_mask = 255-mask
        keypoints = self.detector.detect(reverse_mask)

        img_with_keypoints = cv2.drawKeypoints(raw, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        drawn_msg = self.bridge.cv2_to_imgmsg(img_with_keypoints,"bgr8")
        self.detect_pub.publish(drawn_msg)
                
def main():
    detect = duckie_detector_node()
    rospy.init_node('duckie_detector_node')
    rate = rospy.Rate(10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

