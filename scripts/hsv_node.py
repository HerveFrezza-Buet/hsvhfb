#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from dynamic_reconfigure.server import Server
from hsvhfb.cfg import SelectionParamConfig

class ImageProcessing:
    
    def __init__(self):
        self.hue     = 0
        self.min_sat = .1
        self.min_val = .1
        self.tolerance = .1
        self.select  = False
        
        #### end the initialization with the ros stuff ####
        self.sub    = rospy.Subscriber("image_in/compressed",  CompressedImage, self.on_image,  queue_size = 1)
        self.pub    = rospy.Publisher ("image_out/compressed", CompressedImage,                 queue_size = 1)
        self.srv    = Server(SelectionParamConfig, self.on_reconf)

    def on_reconf(self, config, level):
        self.hue       = config['hue']
        self.min_sat   = config['min_sat']
        self.min_val   = config['min_val']
        self.tolerance = config['tolerance']
        self.select    = config['select']
        return config
        
        
    def on_image(self, ros_data):

        #### From ros message to cv image ####
        compressed_in = np.fromstring(ros_data.data, np.uint8)
        # image_in      = cv2.imdecode(compressed_in, cv2.CV_LOAD_IMAGE_COLOR) --> obsolete
        image_in      = cv2.imdecode(compressed_in, cv2.IMREAD_COLOR)
        width         = image_in.shape[1]
        height        = image_in.shape[0]
        
        #### Processing ####

        image_out = None
        hsv = cv2.cvtColor(image_in, cv2.COLOR_BGR2HSV)
        
        if(self.select):
            image_out = image_in
            hue_min = int(max(self.hue - self.tolerance, 0)*180)
            hue_max = int(min(self.hue + self.tolerance, 1)*180)
            sat_min = int(self.min_sat*255)
            val_min = int(self.min_val*255)
            mask = np.logical_or(hsv[:,:,0] < hue_min, hsv[:,:,0] > hue_max)
            mask = np.logical_or(mask, hsv[:,:,1] < sat_min)
            mask = np.logical_or(mask, hsv[:,:,2] < val_min)
            image_out[mask] = 0
            pass
        else:
            hsv[...,1] = 255
            hsv[...,2] = 255
            image_out = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            

        #### Publishing the result ####
        msg              = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format       = "jpeg"
        msg.data         = np.array(cv2.imencode('.jpg', image_out)[1]).tostring()
        self.pub.publish(msg)

    
if __name__ == '__main__':
    rospy.init_node('demo', anonymous=True)
    try:
        image_processing = ImageProcessing()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down demo_cv_py/demo.py"
