from nav_msgs.msg import Odometry
import tf.transformations as t

import tf
import tf2_ros
import geometry_msgs.msg as geometry_msg
import rospy
import numpy as np
import PyKDL

class TfBroadcaster:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.transform_tolerance = rospy.Duration.from_sec(0.1)


    def send(self, msg, time):
        pose = [msg.x, msg.y, 0.0]
        q = t.quaternion_from_euler(0.0, 0.0, msg.z)
        map_baselink_kdl_vector = PyKDL.Vector(msg.x, msg.y, 0.0)
        map_baselink_kdl_rotation = PyKDL.Rotation.Quaternion(q[0], q[1], q[2], q[3])
        map_baselink_kdl = PyKDL.Frame(map_baselink_kdl_rotation, map_baselink_kdl_vector)
        baselink_map_kdl = map_baselink_kdl.Inverse()

        (trans,rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        baselink_odom_kdl_vector = PyKDL.Vector(trans[0], trans[1], 0.0)
        baselink_odom_kdl_rotation = PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3])
        baselink_odom_kdl = PyKDL.Frame(baselink_odom_kdl_rotation, baselink_odom_kdl_vector)

        odom_map_kdl = baselink_odom_kdl * baselink_map_kdl
        map_odom_kdl = odom_map_kdl.Inverse()

        pose = map_odom_kdl.p
        q = map_odom_kdl.M.GetQuaternion() 

        map_odom_tf_msg = geometry_msg.TransformStamped()
        map_odom_tf_msg.transform.translation.x = pose[0]
        map_odom_tf_msg.transform.translation.y = pose[1]
        map_odom_tf_msg.transform.translation.z = pose[2]
        map_odom_tf_msg.transform.rotation.x = q[0]
        map_odom_tf_msg.transform.rotation.y = q[1]
        map_odom_tf_msg.transform.rotation.z = q[2]     
        map_odom_tf_msg.transform.rotation.w = q[3]
        map_odom_tf_msg.header.stamp = time + self.transform_tolerance
        map_odom_tf_msg.header.frame_id = "map"
        map_odom_tf_msg.child_frame_id = "odom"
        self.br.sendTransformMessage(map_odom_tf_msg)