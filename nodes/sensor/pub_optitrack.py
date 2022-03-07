#!/usr/bin/env python2.7

# -*- coding: utf-8 -*-
# -*- encoding: utf8 -*-
# -*- decoding: utf-8 -*-


import rospy
import threading
from socket import *
from geometry_msgs.msg import Vector3

R = [
 [1, 0, 0],
 [0, 1, 0],
 [0, 0, 1]
]

T = [0.0, 1500.0, 0.0]

class OptitrackListener:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.msg = Vector3()
        self.pub = rospy.Publisher('/robot_pose/optitrack', Vector3, queue_size=10)

    def listener(self):
        while not rospy.is_shutdown():
            try:
                self.socket = socket(AF_INET, SOCK_STREAM)
                self.socket.connect((self.ip, self.port))


                msg = '\t'.join([str(0), str(0), str(0)])
                self.socket.send(msg.encode())
                data = self.socket.recv(1024)
                position = data.split(',')

                x = float(position[0])
                y = float(position[1])
                z = float(position[2])

                self.msg.x = (x + T[0]) / 1000.0
                self.msg.y = (y + T[1]) / 1000.0
                self.msg.z = (z + T[2]) / 1000.0
                self.pub.publish(self.msg)
                print(self.msg)

                rospy.sleep(0.1)

                self.socket.close()
            except:
                self.socket.close()
                pass


    def on(self):
        print("optitrack thread")
        threading.Thread.daemon = True
        self.optitrack_thread = threading.Thread(target=self.listener)
        self.optitrack_thread.start()

if __name__ == '__main__':
    print("Node: Optitrack publisher")
    rospy.init_node('pub_optitrack', anonymous=True)
    rate = rospy.Rate(30)

    socket_ip = '192.168.112.203'
    socket_port = 5000
    optitrack_listener = OptitrackListener(socket_ip, socket_port)
    optitrack_listener.on()
    rospy.spin()

