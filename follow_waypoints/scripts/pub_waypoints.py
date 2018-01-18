#!/usr/bin/env python
# license removed for brevity

import threading
import rospy

from geometry_msgs.msg import Point, Pose, PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import Empty

def callback(msg):
    pub = rospy.Publisher('waypoint', PoseWithCovarianceStamped, queue_size=100)

    data = PoseWithCovarianceStamped()
    data.header.frame_id = 'map'

    data.pose.pose.position.x = msg.position.x
    data.pose.pose.position.y = msg.position.y
    data.pose.pose.position.z = msg.position.z

    data.pose.pose.orientation.x = msg.orientation.x
    data.pose.pose.orientation.y = msg.orientation.y
    data.pose.pose.orientation.z = msg.orientation.z
    data.pose.pose.orientation.w = msg.orientation.w

    pub.publish(data)

def pub_waypoint():
    rospy.init_node('robot_pose', anonymous=True)
    rospy.Subscriber("/robot_pose_throttle", Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        pub_waypoint()
    except rospy.ROSInterruptException:
        pass

