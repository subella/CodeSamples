#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError
import time
from std_msgs.msg import Float64

def image_callback(msg):
    global count
    global start_time
    elapsed_time = time.time() - start_time
    count += 1
    hz = 1/elapsed_time
    pub.publish(hz)
    start_time = time.time()
    
def main():
    image_topic = "/target_cam/depth/image_rect_raw"
    rospy.Subscriber(image_topic, ImageMsg, image_callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('hz_test')
    pub = rospy.Publisher('hz', Float64)
    start_time = time.time()
    count = 0
    main()
