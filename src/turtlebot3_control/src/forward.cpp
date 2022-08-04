#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//! Changeable parameters
double vel = 1;
double r_min = 2;
double r_max = 4;
int rate = 30;
double movtTime = 0.5;

//! Enumerating the movement of the bot into 2 ccategories ->
//! 1. Moving forward
//! 2. Aligning it's direction of motion
enum FSM {
    ALIGN,
    MOVE,
    UPDATE
};

//! Global variables
int movtTimer = rate * movtTime;
double m = (r_max+r_min) * vel / (r_max-r_min);
double c = -2 * vel * (r_max*r_min) / (r_max-r_min);
FSM state = FSM::UPDATE;

//! Generating function, g(r)
double g(double r){
    return (m*r + c);
}

//! Stop the movement of the bot
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
void stopMoving(){
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    return;
}

//! Calculating the roll, pitch and yaw of the bot
tf::Quaternion q;
double roll;
double pitch;
double yaw;
void updateRPY(const geometry_msgs::Quaternion& orientation){
    q.setW(orientation.w);
    q.setX(orientation.x);
    q.setY(orientation.y);
    q.setZ(orientation.z);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return;
}

//! Updating alpha, i.e. the direction the bot should be facing
double alpha = -M_PI_2;
double phi = -M_PI_2;
double x, y, theta, r, slope;
void updateAlpha(const geometry_msgs::Point& position){
    x = position.x;
    y = position.y;
    r = sqrt(x*x + y*y);
    if (r < r_min)
        r = r_min;
    else if (r > r_max)
        r = r_max;
    theta = atan(position.y/position.x);
    slope = g(r) / r;
    phi = asin(-slope/vel);
    alpha = -phi + theta;
    if (alpha < -M_PI){
        while (alpha < -M_PI)
            alpha += M_PI;
    } else if (alpha > M_PI){
        while (alpha > M_PI)
            alpha -= M_PI;
    }
    return;
}

//! Callback to the 'odom' topic
//! Also, the controller for the movement of the bot
double angleDiff = 0;
double K = 0.1;
double Kp = 0.05;
double Ki = 0;
double Kd = 0;
void odomCallback(const nav_msgs::OdometryConstPtr& msg){
    stopMoving();
    updateRPY(msg->pose.pose.orientation);

    if (state == FSM::UPDATE){
        updateAlpha(msg->pose.pose.position);
        state = FSM::ALIGN;
    } else if (state == FSM::ALIGN){
        if (abs(alpha-yaw) > 0.003){
            angleDiff = abs(alpha-yaw);
            if (angleDiff > M_PI){
                if (alpha > yaw)
                    cmd_vel.angular.z = -K - angleDiff*Kp;
                else
                    cmd_vel.angular.z = K + angleDiff*Kp;
            } else {
                if (alpha > yaw)
                    cmd_vel.angular.z = K + angleDiff*Kp;
                else
                    cmd_vel.angular.z = -K - angleDiff*Kp;
            }
        } else {
            state = FSM::MOVE;
        }
    } else if (state == FSM::MOVE){
        if (movtTimer--)
            cmd_vel.linear.x = vel / rate;
        else {
            movtTimer = rate * movtTime;
            state = FSM::UPDATE;
        }
    }
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "forward");
    ros::NodeHandle nh;

    ros::Publisher pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", rate);
    ros::Subscriber sub_odom_ = nh.subscribe<nav_msgs::Odometry>("odom", rate, odomCallback);
    ros::Rate loopRate(rate);

    while (ros::ok()){
        ros::spinOnce();
        pub_cmd_vel_.publish(cmd_vel);
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}