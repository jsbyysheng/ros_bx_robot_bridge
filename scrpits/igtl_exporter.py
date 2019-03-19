#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import math
import tf
import geometry_msgs.msg
from ros_igtl_bridge.msg import igtlstring
from ros_igtl_bridge.msg import igtltransform
from geometry_msgs.msg import Transform


def push_transform(pub, name, trans, rot):

    transform = Transform()
    transform.translation.x = trans[0] * 1000
    transform.translation.y = trans[1] * 1000
    transform.translation.z = trans[2] * 1000
    transform.rotation.x = rot[0]
    transform.rotation.y = rot[1]
    transform.rotation.z = rot[2]    
    transform.rotation.w = rot[3]
    
    transmsg = igtltransform()
    transmsg.name = name
    transmsg.transform = transform

    pub.publish(transmsg)
    

def igtl_exporter():

    rospy.init_node('igtl_exporter', anonymous=True)

    pub_igtl_transform_out = rospy.Publisher('IGTL_TRANSFORM_OUT', igtltransform, queue_size=10)    

    listener = tf.TransformListener()
    
    #pub = rospy.Publisher('igtl_exporter', String, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    hello_str = "hello world %s" % rospy.get_time()

    while not rospy.is_shutdown():
        
        try:
            (trans1,rot1) = listener.lookupTransform('base_link', 'shoulder_link', rospy.Time())
            (trans2,rot2) = listener.lookupTransform('base_link', 'upper_arm_link', rospy.Time())
            (trans3,rot3) = listener.lookupTransform('base_link', 'forearm_link', rospy.Time())
            (trans4,rot4) = listener.lookupTransform('base_link', 'wrist_1_link', rospy.Time())
            (trans5,rot5) = listener.lookupTransform('base_link', 'wrist_2_link', rospy.Time())
            (trans6,rot6) = listener.lookupTransform('base_link', 'wrist_3_link', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        push_transform(pub_igtl_transform_out, 'shoulder_link', trans1, rot1)
        push_transform(pub_igtl_transform_out, 'upper_arm_link', trans2, rot2)
        push_transform(pub_igtl_transform_out, 'forearm_link', trans3, rot3)
        push_transform(pub_igtl_transform_out, 'wrist_1_link', trans4, rot4)
        push_transform(pub_igtl_transform_out, 'wrist_2_link', trans5, rot5)
        push_transform(pub_igtl_transform_out, 'wrist_3_link', trans6, rot6)

        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()
        

if __name__ == '__main__':

    igtl_exporter()


