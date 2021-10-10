#! /usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2


class TurtleChasing:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        
        rospy.Subscriber('/turtle1/pose', Pose, self.run)
        rospy.Subscriber('/turtle_chaser/pose', Pose, self.get_curr)
        self.pub = rospy.Publisher('/turtle_chaser/cmd_vel', Twist, queue_size=10)

    def run(self, msg):
        x = msg.x
        y = msg.y
        diff_x = x - self.x
        diff_y = y - self.y

        msg_chaser = Twist()
        msg_chaser.angular.z = - self.theta + atan2(diff_y, diff_x + 1e-8)
        msg_chaser.linear.x = 2 * diff_x
        msg_chaser.linear.y = 2 * diff_y
        self.pub.publish(msg_chaser)
        
    def get_curr(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta


if __name__ == '__main__':
    rospy.init_node('turtle_chase')
    TurtleChasing()
    rospy.spin()
