#!/usr/bin/python

from visualization_msgs.msg import Marker
import rospy

rospy.init_node('marker_publisher', anonymous=True)
publisher = rospy.Publisher("robot_marker", Marker, queue_size=10)
count = 0
while not rospy.is_shutdown():
    ellipse = Marker()
    ellipse.header.frame_id = "map"
    ellipse.header.stamp = rospy.Time.now()
    ellipse.type = Marker.CYLINDER
    ellipse.pose.position.x = 0
    ellipse.pose.position.y = 0
    ellipse.pose.position.z = 0
    ellipse.pose.orientation.x = 0
    ellipse.pose.orientation.y = 0
    ellipse.pose.orientation.z = 0
    ellipse.pose.orientation.w = 1
    ellipse.scale.x = 2
    ellipse.scale.y = 2
    ellipse.scale.z = 1
    ellipse.color.a = 1.0
    ellipse.color.r = 1.0
    ellipse.color.g = 1.0
    ellipse.color.b = 1.0

    # Publish the MarkerArray
    publisher.publish(ellipse)

    rospy.sleep(0.01)
