#include <iostream>
#include <fstream>
#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

enum FSM {
    MOVE,
    ALIGN,
    TRAJECTORY
};

const int RATE = 30;
const double TOLERANCE = 0.001;
const double INITIAL_ANGLE = M_PI_2;

int current = 1;
double roll, pitch, yaw;

tf::Quaternion q;
nav_msgs::Path path = nav_msgs::Path();
std::vector<geometry_msgs::Point> waypoints;
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
geometry_msgs::Point fileWaypoint = geometry_msgs::Point();

FSM state = FSM::ALIGN;

void addToPath(const nav_msgs::OdometryConstPtr& msg){
    path.header = msg->header;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path.poses.push_back(pose);
    return;
}

void stopMoving(){
    cmd_vel.linear.x = 0;
    cmd_vel.angular.y = 0;
    return;
}

void updateRPY(const geometry_msgs::Quaternion& orientation){
    q.setW(orientation.w);
    q.setX(orientation.x);
    q.setY(orientation.y);
    q.setZ(orientation.z);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return;
}

int getDirection(double theta){
    if (abs(theta-yaw)<0.1 || abs(M_PI*2+theta-yaw)<0.1 || abs(M_PI*2-theta+yaw)<0.1)
        return 0;
    
    if (theta > yaw){
        if (theta>M_PI_2 && yaw<-M_PI_2)
            return -1;
        else
            return 1;
    } else {
        if (theta<-M_PI_2 && yaw>M_PI_2)
            return 1;
        else
            return -1;
    }
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg){
    stopMoving();
    addToPath(msg);
    updateRPY(msg->pose.pose.orientation);
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (state == FSM::MOVE){
        cmd_vel.linear.x = 0.05;
        if (abs(waypoints[0].x-x) < TOLERANCE){
            state = FSM::ALIGN;
            ROS_WARN("\tMovement done. Aligning now...");
        }
        return;
    }

    if (state == FSM::ALIGN){
        double angleDiff = INITIAL_ANGLE - yaw;
        if (abs(angleDiff) > 0.003){
            if (angleDiff > 0)
                cmd_vel.angular.z = 0.1 + 0.04*angleDiff;
            else
                cmd_vel.angular.z = -0.1 + 0.04*angleDiff;
        } else {
            state = FSM::TRAJECTORY;
            ROS_WARN("\tAlignment done. Generating trajectory now..");
        }
        return;
    }

    if (state == FSM::TRAJECTORY){
        if (current == waypoints.size()){
            return;
        }
        double finalX = waypoints[current].x;
        double finalY = waypoints[current].y;
        double dist = sqrt(pow(finalX-x,2) + pow(finalY-y,2));
        if (dist < 0.02){
            ROS_INFO("\tReached waypoint #%d with a distance of %f", current, dist);
            current++;
            return;
        }
        double theta = atan2(finalY-y, finalX-x);
        cmd_vel.angular.z = getDirection(theta) * 0.7;
        cmd_vel.linear.x = 0.1;
        return;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "trial");
    ros::NodeHandle nh;

    fileWaypoint.z = 0;
    std::fstream file;
    std::string line;
    file.open("/home/adit/unicycle-model-patterns/src/turtlebot3_follower/path/waypoints.txt", std::ios::in);
    if (file.is_open()){
        std::string x, y;
        while (getline(file, line)){
            std::stringstream ss(line);
            ss >> x >> y;
            fileWaypoint.x = std::stod(x);
            fileWaypoint.y = std::stod(y);
            waypoints.push_back(fileWaypoint);
        }
        ROS_WARN("No. of waypoints read = %lu", waypoints.size());
        ROS_WARN("Target Point: (%f, %f)", waypoints[0].x, waypoints[0].y);
        file.close();
    } else {
        ROS_ERROR("Couldn't open file");
        return EXIT_FAILURE;
    }

    ros::Publisher pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", RATE);
    ros::Publisher pub_path_ = nh.advertise<nav_msgs::Path>("path", RATE);
    ros::Subscriber sub_odom_ = nh.subscribe<nav_msgs::Odometry>("odom", RATE, odomCallback);
    ros::Rate loopRate(RATE);

    while (ros::ok()){
        ros::spinOnce();
        pub_cmd_vel_.publish(cmd_vel);
        pub_path_.publish(path);
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}