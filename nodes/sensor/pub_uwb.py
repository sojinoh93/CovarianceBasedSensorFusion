#!/usr/bin/env python2.7

# -*- coding: utf-8 -*-
# -*- encoding: utf8 -*-
# -*- decoding: utf-8 -*-

import rospy
import paho.mqtt.client as mqtt
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
import numpy as np

pub_uwb = rospy.Publisher("/robot_pose/uwb", Odometry, queue_size=100)

def callback(client, userdata, uwb_msg):
    data = eval(str(uwb_msg.payload.decode("utf-8")))
    x = float(data['position']['x'])
    y = float(data['position']['y'])
    z = float(data['position']['z'])    

    if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
        sn = 1e-10  # small_number number
        ln = 1e+10  # large number
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position = Vector3(x, y, z)
        msg.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        msg.pose.covariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.05, 0.0, 0.0, 0.0, 0.0,  
                                0.0, 0.0, ln, 0.0, 0.0, 0.0,  
                                0.0, 0.0, 0.0, ln, 0.0, 0.0,  
                                0.0, 0.0, 0.0, 0.0, ln, 0.0,  
                                0.0, 0.0, 0.0, 0.0, 0.0, ln]
        # msg.twist.covariance = [ln, 0.0, 0.0, 0.0, 0.0, 0.0,
        #                         0.0, ln, 0.0, 0.0, 0.0, 0.0,  
        #                         0.0, 0.0, ln, 0.0, 0.0, 0.0,  
        #                         0.0, 0.0, 0.0, ln, 0.0, 0.0,  
        #                         0.0, 0.0, 0.0, 0.0, ln, 0.0,  
        #                         0.0, 0.0, 0.0, 0.0, 0.0, ln]
        pub_uwb.publish(msg)
        
if __name__ == '__main__':
    print("Node: UWB publisher")
    rospy.init_node('pub_uwb', anonymous=True)

    ip = '192.168.112.180'
    topic = 'dwm/node/023a/uplink/location'

    client = mqtt.Client()
    client.connect(ip, 1883)
    client.subscribe(topic)
    client.on_message = callback

    while not rospy.is_shutdown():
        client.loop_start()

