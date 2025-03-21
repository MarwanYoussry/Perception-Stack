#!/usr/bin/env python3
import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from obj_detection.msg import Seg,detect_list
from obj_detection.msg import tracker_res,tracker_list
from yolox.tracker.byte_tracker import BYTETracker

#BYTETracker settings
class args:
        track_thresh = 0.1
        track_buffer = 30
        match_thresh = 0.3
        mot20 = False

bridge=CvBridge()
tracker = BYTETracker(args)


def detection_callback(detect_list):
    
    
    frame_id=detect_list.header.frame_id 
    
    #creating custom frame
    #width, height = 1280, 720  
    #black_screen = np.zeros((height, width, 3), dtype=np.uint8)    
    
    tracker_msg=tracker_list()
    tracker_msg.header = detect_list.header

    ######### Extracting info from detect_list_msg ######### 
    
    detections=[]
    for detect in detect_list.detect:
    
        x1,y1,x2,y2=detect.x1,detect.y1,detect.x2,detect.y2
        conf,img_info=detect.conf,detect.img_info
        class_name,class_id=detect.class_name,detect.class_id
        #rospy.loginfo(f"Tracking on frame{frame_id}! ")
        detections.append([x1,y1,x2,y2,conf,class_id])
   
    ########## Preparing data for Bytetracker ########
    img_size=[img_info[0],img_info[1]]
    img_size=tuple(img_size)
    detect_np=np.array(detections,dtype=np.float32)
    detect_tensor = torch.tensor(detect_np, dtype=torch.float32)
    #rospy.loginfo(f"img_size:, {img_size}, Type: {type(img_size)}")
    #rospy.loginfo(f"img_size[0] Type:, {type(img_size[0])}, img_size[1] Type:, {type(img_size[1])}")
   
    ############ Bytetracker Model #########
    tracker_result= tracker.update(detect_tensor,img_info,img_size)
    #rospy.loginfo(f"{tracker_result}")
    ########### Extracting result #########
    for obj in tracker_result:
        #rospy.loginfo(f"Track ID: {obj.track_id}, Bounding Box: {obj.tlbr}, Confidence: {obj.score}")

        x1, y1, x2, y2 = map(int, obj.tlbr)  # Bounding box
        #rospy.loginfo(f"frame_id {frame_id} bbox:{x1}, {y1}, {x2}, {y2}")
        
        res_info = tracker_res()
        res_info.x1 = x1
        res_info.y1 = y1
        res_info.x2 = x2
        res_info.y2 = y2
        res_info.class_name=class_name
        res_info.conf = float(obj.score)
        res_info.img_info=img_info
        res_info.track_id=int(obj.track_id)
        tracker_msg.tracker.append(res_info)

        
        # testing results on custom frame
        #cv2.rectangle(black_screen, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #cv2.putText(black_screen, f"ID: {obj.track_id} {class_name}", (x1, y1 - 10),
               #cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    #cv2.imshow("Tracking", black_screen)
    #cv2.waitKey(1)
    tracker_pub.publish(tracker_msg)


if __name__=="__main__":
    node_name ="tracker"
    seg_topic ="/seg_list"
    tracker_topic="/tracker_list"
    tracker_pub =rospy.Publisher(f'{tracker_topic}',tracker_list,queue_size=500) 
    rospy.init_node(node_name)
    rospy.Subscriber(f'{seg_topic}', detect_list,detection_callback)

    

    rospy.loginfo(f"{node_name} is created & Is sub to {seg_topic}")
    rospy.spin()