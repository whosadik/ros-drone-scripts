#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import time

def main():
    rospy.init_node("target_generator_node")

    message_pub = rospy.Publisher("/target_position", PoseStamped, queue_size=10)

    rate = rospy.Rate(2)

    waypoints = [
        (0, 0, 2),
        (2, 0, 2),
        (2, 2, 2),
        (0, 2, 2)
    ]

    i = 0

    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'

        pose.pose.position.x = waypoints[i][0]
        pose.pose.position.y = waypoints[i][1]
        pose.pose.position.z = waypoints[i][2]

        message_pub.publish(pose)

        rospy.loginfo("Цель: x=%.1f y=%.1f z=%.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)

        i = (i+1)% len(waypoints)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        

    