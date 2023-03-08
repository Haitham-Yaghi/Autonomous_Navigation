#!/usr/bin/env python

# aim is to subscribe to /m2wr/laser/scan  or  use the data for obstacle avoidance or then publish a Twist message to give the bot command to move

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class obstacle_avoid():


    def __init__(self):
        self.sub = rospy.Subscriber("/scan", LaserScan , self.callback)
        self.case = ''

    def checkcase(self,range): 
    
        if ( range["right"] > 0.9  and range["center"] < 0.9  and range["left"] < 0.9  ):
            self.case = 'OBSTACLE RIGHT!'

        elif ( range["right"] < 0.9   and range["center"] < 0.9  and range["left"] > 0.9 ):
            self.case = 'OBSTACLE LEFT!'

        elif ( range["right"] < 0.9   and range["center"] < 0.9  and range["left"] < 0.9  ):
            self.case = 'OBSTACLE CENTER!'
        
        else: 
            self.case = 'NO OBSTACLE!'
            
        
        rospy.loginfo(self.case)
        
    def getCase(self):
        return self.case

    def callback(self,message):
        l=len(message.ranges)
        e = l/3
        range={
            "right" : min(min(message.ranges[0:e-1]) , 1),
            "center" : min(min(message.ranges[e:(2*e-1)]) , 1),
            "left" : min(min(message.ranges[(2*e):(3*e-1)]) , 1)
        }

        self.checkcase(range)

    # def listener():
    #     rospy.init_node('obstacle_avoid', anonymous=False) #This should be in the main node
    #     #sub = rospy.Subscriber("/scan", LaserScan , callback)
    #     rospy.spin()


class Printer():
    def __init__(self):



def main():
    OB=obstacle_avoid()
    rospy.init_node('obstacle_avoid', anonymous=False) #This should be in the main node
    rospy.spin()



if __name__ == '__main__':
   main()