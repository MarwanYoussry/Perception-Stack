#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from obj_detection.msg import Seg,detect_list 
from yolox.tracker.byte_tracker import BYTETracker

class args:
        track_thresh = 0.5
        track_buffer = 30
        match_thresh = 0.8 
        mot20 = False

bridge=CvBridge()
tracker = BYTETracker(args)
def detection_callback(detect_list):
     img_info=(detect_list.header.height,) 
    rospy.loginfo(f"Tracking is running on frame {detect_list.detect} ! ")
    frame_id=detect_list.header.frame_id 
    
    
    for detect in detect_list.detect:
    
        x1,y1,x2,y2,conf,class_name=detect.x1,detect.y1,detect.x2,detect.y2,detect.conf,detect.class_name
        #rospy.loginfo(f"equal to {x1}, {y1}, {x2}, {y2} .{conf}, {class_name}! ")
        detect=[x1,y1,x2,y2,conf]
    
    detect_np=np.array(detect,dtype=np.float32)
    tracker_result= tracker.update(detect_np,,)








if __name__=="__main__":
    node_name ="tracker"
    seg_topic ="/seg_list"
    tracker_topic="/tracker"
    #Seg_pub =rospy.Publisher(f'{seg_topic}',detect_list,queue_size=1000) 
    rospy.init_node(node_name)
    rospy.Subscriber(f'{seg_topic}', detect_list,detection_callback)

    

    rospy.loginfo(f"{node_name} is created & Is sub to {seg_topic}")
    rospy.spin()