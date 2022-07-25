#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def pos_cb(data: Odometry):
    print('Position:')
    print('    x: ', data.pose.pose.position.x)
    print('    y: ', data.pose.pose.position.y)
    print('    z: ', data.pose.pose.position.z)
    print('')

def main():
    rospy.init_node('forward', anonymous=True)
    rate = rospy.Rate(10)
    
    pub_vel = rospy.Publisher('cmd_vel', Twist)
    sub_odom = rospy.Subscriber('odom', Odometry, pos_cb)

    vel = Twist()
    vel.linear.x = 0.1

    while not rospy.is_shutdown():
        pub_vel.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass