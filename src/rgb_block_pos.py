#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


class cImageHandler:

    def __init__(self):
        self.point_pub = rospy.Publisher("/rgb_seg/block_location", Point, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)


    def callback(self, data):
        sGauss = 5
        sThresh = 100

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        s = hsv_image[:,:,1]
        

        sGauss = cv2.GaussianBlur( s, (sGauss, sGauss), 0 )
        _, threshS = cv2.threshold(sGauss, sThresh, 255, cv2.THRESH_BINARY)
        threshS = self.draw_over_smallest_blobs(threshS)

        block = self.get_largest_blob(threshS)
        if block == None:
            print "get_largest_blob returned None"
            return

        M = cv2.moments(block)
        centroidx = int(M['m10']/M['m00'])
        centroidy = int(M['m01']/M['m00'])

        # print "centroid:  ", centroidx, centroidy, "\n"

        # cv2.imshow("RGB Image", cv_image)
        # cv2.imshow("S Image", s)
        cv2.imshow("S Thresholded", threshS)
        cv2.waitKey(3)

        p = Point(centroidx, centroidy, 0)
        self.point_pub.publish(p)


    def draw_over_smallest_blobs(self, image):
        copyOfImage = image.copy()

        # We want to keep the biggest blob so long as it isn't too big.
        biggestIndex = -1
        biggestArea = 0.0
        contours, _ = cv2.findContours(copyOfImage, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if contours.__len__() > 0:
            for i in range(contours.__len__()):
                c_area = cv2.contourArea(contours[i])
                if c_area > biggestArea:
                    biggestIndex = i
                    biggestArea = c_area
                    
        if biggestIndex != -1:
            for i in range(0,contours.__len__()):
                if i != biggestIndex:
                    cv2.drawContours(image, contours, i, (0,0,0), -1, cv2.CV_AA)
        
        return image


    def get_largest_blob(self, image):
        copyOfImage = image.copy()

        biggestIndex = -1
        biggestArea = 0.0
        contours, _ = cv2.findContours(copyOfImage, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if contours.__len__() > 0:
            for i in range(contours.__len__()):
                c_area = cv2.contourArea(contours[i])
                if c_area > biggestArea:
                    biggestIndex = i
                    biggestArea = c_area
        else:
            return None

        return contours[biggestIndex]        


def main(args):
    handler = cImageHandler()
    rospy.init_node("rgb_block_pos", anonymous=True)

    rospy.spin()
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
