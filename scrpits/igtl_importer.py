#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ROS_IGTL_Bridge.msg import igtlstring
from ROS_IGTL_Bridge.msg import igtltransform

def igtl_importer():

    global pub_igtl_transform_out
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pub_igtl_transform_out = rospy.Publisher('IGTL_TRANSFORM_OUT', igtltransform, queue_size=10)
    
    rospy.init_node('st_command_listener', anonymous=True)
    rospy.Subscriber("IGTL_TRANSFORM_IN", igtltransform, cb_transform)
    rospy.Subscriber("IGTL_STRING_IN", igtlstring, cb_string)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    igtl_importer()



