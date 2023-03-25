#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

msg = Twist()
case = ''
msg.linear.x = 0
msg.angular.z = 0

def checkcase(range): 
   
    if ( range["right"] > 0.9  and range["center"] < 0.9  and range["left"] < 0.9  ):
        msg.linear.x =0.3
        msg.angular.z=0.4
        pub.publish(msg)
        case = 'OBSTACLE RIGHT!'

    elif ( range["right"] < 0.9   and range["center"] < 0.9  and range["left"] > 0.9 ):
        msg.linear.x=0.3
        msg.angular.z=-0.4
        pub.publish(msg)
        case = 'OBSTACLE LEFT!'

    elif ( range["right"] < 0.9   and range["center"] < 0.9  and range["left"] < 0.9  ):
        msg.linear.x= 0    
        msg.angular.z=-0.4
        pub.publish(msg)
        case = 'OBSTACLE CENTER!'
       
    else: 
        msg.linear.x =0.4
        msg.angular.z=0
        pub.publish(msg)
        case = 'NO OBSTACLE!'
        
        

    rospy.loginfo(case)
    


def callback(message):
    l=len(message.ranges)
    e = l/3
    range={
        "right" : min(min(message.ranges[0:e-1]) , 1),
        "center" : min(min(message.ranges[e:(2*e-1)]) , 1),
        "left" : min(min(message.ranges[(2*e):(3*e-1)]) , 1)
    }

    checkcase(range)

def listener():
    global pub
    rospy.init_node('obstacle_avoid', anonymous=False)
    pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=1)
    sub = rospy.Subscriber("/scan", LaserScan , callback)
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    
    rospy.spin()    

if __name__ == '__main__':
    listener()
