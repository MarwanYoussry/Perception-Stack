#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from obj_detection.msg import tracker_res,tracker_list

prev_time=time.time()
bridge =CvBridge()
def fusion_callback (cam_msg, tracker_msg,flow_msg):
    global bridge, prev_time
    
    frame =bridge.imgmsg_to_cv2(cam_msg,desired_encoding="bgr8")
    flow =bridge.imgmsg_to_cv2(flow_msg,desired_encoding="32FC2")
    #### Motion vectors (dx & dy) #####
    flow_x=flow[...,0]
    flow_y=flow[...,1]
    curr_time=time.time()
    if(curr_time!=prev_time):
        fps=1/(curr_time-prev_time)
        cv2.putText(frame, f"FPS: {fps:.2f}", (0,20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    prev_time=curr_time
    rospy.loginfo(f"frame {cam_msg.header.frame_id}")
    for obj in tracker_msg.tracker:
        x1,y1,x2,y2=obj.x1,obj.y1,obj.x2,obj.y2
        conf=obj.conf
        class_name,track_id=obj.class_name,obj.track_id
        if flow_x[y1:y2, x1:x2].size >0:
            avg_dx= np.mean(flow_x[y1:y2, x1:x2])
            avg_dy= np.mean(flow_y[y1:y2, x1:x2])
        else:
            avg_dx,avg_dy=0,0
        speed = np.sqrt(avg_dx**2 + avg_dy**2)*fps


        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"ID: {obj.track_id} Class_Name: {class_name}", (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, f"speed : {speed:.2f}", (x1, y1 +10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    fusion=bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    fusion_pub.publish(fusion)    
    #cv2.imshow("Processed Img",frame)
    #cv2.waitKey(50)

if __name__=="__main__":
    node_name ="fusion"
    vid_topic ="/video"
    tracker_topic="/tracker_list"
    opt_topic ="/optical_img"
    fusion_topic="/final_res"

    rospy.init_node(node_name)
    fusion_pub =rospy.Publisher(f'{fusion_topic}',Image,queue_size=500)
    cam_sub=message_filters.Subscriber(f'{vid_topic}', Image)
    tracker_sub=message_filters.Subscriber(f'{tracker_topic}', tracker_list)
    flow_sub=message_filters.Subscriber(f'{opt_topic}', Image)    

    sync=message_filters.ApproximateTimeSynchronizer([cam_sub, tracker_sub,flow_sub],queue_size=1000,slop=0.2)
    sync.registerCallback(fusion_callback)
    
    rospy.loginfo(f"{node_name} is created & Is sub to {vid_topic} & {tracker_topic} & {opt_topic}")
    rospy.spin()