#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class camera_sync_node():

    def __init__(self):
        self.concat_pub = rospy.Publisher('/cameraduckie/concat_images', Image, queue_size=10)
        self.bridge = CvBridge()

        #subscribers for the  
        self.d_img_sub = message_filters.Subscriber('/cameraduckie/camera_node/image/compressed', CompressedImage)
        self.d_inf_sub = message_filters.Subscriber('/cameraduckie/camera_node/camera_info', CameraInfo)
        self.c_img_sub = message_filters.Subscriber('/camera/image_color/compressed', CompressedImage)
        self.c_inf_sub = message_filters.Subscriber('/camera/camera_info', CameraInfo)
        self.ats = message_filters.ApproximateTimeSynchronizer([self.d_img_sub,self.c_img_sub], queue_size=5, slop=0.05)

    def callback(self,duckie_image, corner_image):
        #print("Callback!")
        duckie = self.bridge.compressed_imgmsg_to_cv2(duckie_image)
        corner = self.bridge.compressed_imgmsg_to_cv2(corner_image)
        images = [duckie,corner] 
        duckie_y, duckie_x = duckie.shape[:2]
        corner_resized = cv2.resize(corner, (duckie_x,duckie_y))
        
        vis = np.concatenate((duckie,corner_resized), axis=1)
        vis_msg = self.bridge.cv2_to_imgmsg(vis,"bgr8")
        self.concat_pub.publish(vis_msg)
    
    def get_one_image(self,images):

        height = sum(image.shape[0] for image in images)
        width = max(image.shape[1] for image in images)
        output = np.zeros((height,width,3))

        y = 0
        for image in images:
            h,w,d = image.shape
            output[y:y+h,0:w] = image
            y += h

        return output

def main():
    sync = camera_sync_node()
    rospy.init_node('camera_sync_node')
    rate = rospy.Rate(10)
    sync.ats.registerCallback(sync.callback)
    print("Callback registered")

    #puppy test
    #test_img = cv2.imread('test.jpg', 1)
    #print(test_img)
    #img_msg = sync.bridge.cv2_to_imgmsg(test_img, "bgr8")
    #sync.concat_pub.publish(img_msg)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

