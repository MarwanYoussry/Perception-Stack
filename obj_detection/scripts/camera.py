#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def video_stream():
    rospy.init_node("video")
    pub =rospy.Publisher("/video",Image,queue_size=1000)
    bridge =CvBridge()

    video=cv2.VideoCapture("/home/mohamed/catkin_ws/src/obj_detection/scripts/test_video")
    if not video.isOpened():
            rospy.logerr("Failed")
            return
    frame_id=0
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        ret, frame =video.read()

        if not ret :
            rospy.logerr("video is Finished")
            break
        ros_image= bridge.cv2_to_imgmsg(frame ,encoding="bgr8")
        ros_image.header.stamp=rospy.Time.now()
        ros_image.header.frame_id=str(frame_id)
        frame_id=frame_id+1
        pub.publish(ros_image)
        rospy.loginfo(f"Published frame {frame_id}")
        rate.sleep()
    video.release()

if __name__=="__main__":
    try:
        video_stream()
    except rospy.ROSInterruptException:
        pass