#!/usr/bin/env python

import rospy, roslib
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose

lab = Pose()
mark = Marker()

def whycon_pub(data,pubWhycon):
    mark = data.markers
    lab = mark[0].pose
    pubWhycon.publish(lab)


def main_loop() :
    rospy.init_node("whycon_publisher", anonymous=True)
    bebopName = rospy.get_param("~bodyName",'B1')
    pubWhycon = rospy.Publisher('/%s/whycon_lab' % bebopName, Pose, queue_size = 10)
    subWhycon = rospy.Subscriber('/whycon_ros/visual',
                     MarkerArray,
                     whycon_pub,pubWhycon)

    rospy.spin()


if __name__ == "__main__" :
    try:
        main_loop()
    except rospy.ROSInterruptException :
        pass