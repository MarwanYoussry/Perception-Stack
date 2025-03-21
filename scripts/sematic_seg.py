#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from obj_detection.msg import Seg,detect_list 

bridge=CvBridge()
model = YOLO("/home/mohamed/catkin_ws/src/obj_detection/scripts/yolo11n-seg.pt")
def detection_callback(image_msg):
    global bridge
    image_header=image_msg.header
    msg_timestamp=image_header.stamp
    msg_id=image_header.frame_id
    
    frame= bridge.imgmsg_to_cv2(image_msg,desired_encoding="bgr8")

    detection_results= model(frame, conf=0.4, iou=0.4)[0]
    #rospy.loginfo(f"Detection is running on frame {msg_id}! {frame.shape}")
    
    detect_msg =detect_list()
    detect_msg.header = image_header
    
    for mask, box in zip(detection_results.masks.xy,detection_results.boxes):
        
        #mask_np= np.array([mask], dtype= np.int32)
        #cv2.fillPoly(frame, [mask_np], color=(0,255,0))

        x1 , y1 , x2 , y2=map (int , box.xyxy[0])
        conf = float(box.conf[0])
        class_id = int(box.cls[0])
        class_name= model.names[class_id]

        Seg_info = Seg()
        Seg_info.x1 = x1
        Seg_info.y1 = y1
        Seg_info.x2 = x2
        Seg_info.y2 = y2
        Seg_info.conf = conf
        Seg_info.class_name = class_name
        Seg_info.img_info=frame.shape
        Seg_info.class_id=class_id
        detect_msg.detect.append(Seg_info)

        #cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
        #cv2.putText(frame,f'{class_name} {class_id} {int(conf*100)}%',(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0))
    Seg_pub.publish(detect_msg)
    #cv2.imshow("detection",frame)
    #cv2.waitKey(1)
    





if __name__=="__main__":
    vid_topic ="/video"
    node_name ="sematic_seg"
    seg_topic ="/seg_list"
    Seg_pub =rospy.Publisher(f'{seg_topic}',detect_list,queue_size=500) 
    rospy.init_node(node_name)
    rospy.Subscriber(f'{vid_topic}', Image,detection_callback)

    

    rospy.loginfo(f"{node_name} is created & Is sub to {vid_topic}")
    rospy.spin()