#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

tf::Quaternion q;
geometry_msgs::Twist vel = geometry_msgs::Twist();
double roll, pitch, yaw;

//! sub_odom_ call back function that dictates turning of the bot
void odometryCallback(const nav_msgs::OdometryConstPtr& msg){
    //! Quaternion conversion
    q.setW(msg->pose.pose.orientation.w);
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);

    //! Euler angle conversion
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    //! Movement
    if (yaw <= M_PI_2)
        vel.angular.z = 0.1;
    else
        vel.angular.z = 0;

    //! Print on console
    std::cout << "Orientation:" << std::endl;
    std::cout << "    R: " << roll << std::endl;
    std::cout << "    P: " << pitch << std::endl;
    std::cout << "    Y: " << yaw << std::endl;
    std::cout << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "forward");
    ros::NodeHandle nh;

    ros::Publisher pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 30);
    ros::Subscriber sub_odom_ = nh.subscribe<nav_msgs::Odometry>("odom", 30, odometryCallback);
    
    ros::Rate loopRate(30);

    while(ros::ok()){
        ros::spinOnce();

        pub_cmd_vel_.publish(vel);

        loopRate.sleep();
    }
    
    return EXIT_SUCCESS;
}