#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

ARROW_LEFT = 10
ARROW_UP = 47
ARROW_RIGHT = 5

id = 0
pub = None
msg = Twist()

camera_center = 320  # left 0, right 640
max_ang_msg = 0.6
min_ang_msg = 0.4
ang_msg = 0

distFL = 0
distFR = 0
average_dist = 0
desired_dist = 0.2


def distFL_callback(range_msg):
    global distFL
    distFL = range_msg.range


def distFR_callback(range_msg):
    global distFR
    distFR = range_msg.range


def objectCallback(object_msg):
    global id, pub, msg

    if len(object_msg.data) > 0:
        id = object_msg.data[0]

        objectWidth = object_msg.data[1]
        objectHeight = object_msg.data[2]
        speed_coefficient = float(camera_center) / max_ang_msg


        if id == ARROW_LEFT:
            msg.linear.x = 0
            msg.angular.z = 1
        elif id == ARROW_UP:
            msg.linear.x = 0.5
            msg.angular.z = 0
        elif id == ARROW_RIGHT:
            msg.linear.x = 0
            msg.angular.z = -1
        else:  # other object
            msg.linear.x = 0
            msg.angular.z = 0
        pub.publish(msg)
    else:
        # No object detected
        msg.linear.x = 0
        msg.angular.z = 0

    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('action_controller')
    sub = rospy.Subscriber('/objects', Float32MultiArray, objectCallback)

    distL_sub = rospy.Subscriber('/range/fl', Range, distFL_callback)
    distR_sub = rospy.Subscriber('/range/fr', Range, distFR_callback)
    loop_rate = rospy.Rate(50)
    action_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    while not rospy.is_shutdown():
        rospy.spin()
        loop_rate.sleep()
