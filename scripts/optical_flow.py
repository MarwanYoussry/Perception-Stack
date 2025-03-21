#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

prev_frame= None
bridge=CvBridge()
def detection_callback(image_msg):
    global bridge, prev_frame
    image_header=image_msg.header
    msg_id=image_header.frame_id
    #rospy.loginfo(f"current frame {msg_id}")
    frame= bridge.imgmsg_to_cv2(image_msg,desired_encoding="bgr8")
    curr_frame= cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    if prev_frame is not None:
        flow = cv2.calcOpticalFlowFarneback(prev_frame, curr_frame, None,0.5, 3, 15, 3, 5, 1.2, 0)
        flow_msg=bridge.cv2_to_imgmsg(flow, encoding="32FC2")
        flow_msg.header=image_msg.header

        Optical_pub.publish(flow_msg)

        ####### testing#####
        #hsv = np.zeros((flow.shape[0], flow.shape[1], 3), dtype=np.uint8)
        #mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        #hsv[..., 0] = ang * 180 / np.pi / 2
        #hsv[..., 1] = 255
        #hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        #flow_vis = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        #cv2.imshow("Optical Flow", flow_vis)
        #cv2.waitKey(1)

    prev_frame=curr_frame


if __name__=="__main__":
    vid_topic ="/video"
    node_name ="optical_flow"
    opt_topic ="/optical_img"
    Optical_pub =rospy.Publisher(f'{opt_topic}',Image,queue_size=500) 
    rospy.init_node(node_name)
    rospy.Subscriber(f'{vid_topic}', Image,detection_callback)

    

    rospy.loginfo(f"{node_name} is created & Is sub to {vid_topic}")
    rospy.spin()