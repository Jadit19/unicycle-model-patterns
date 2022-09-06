#include<ros/ros.h>
#include<tf/tf.h>
#include<iostream>
#include<fstream>
#include<cmath>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>

enum FSM {
    MOVE,
    ALIGN,
    TRAJECTORY
};

const int RATE = 30;
const double TOLERANCE = 0.001;
const double INITIAL_ANGLE = M_PI_2;
double kp=1.5;
double ki=2;
double kd=0.02;
double dt=0.01;

int current = 1;
double roll, pitch, yaw;

tf::Quaternion q;
nav_msgs::Path path = nav_msgs::Path();
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
std::vector<geometry_msgs::Point> waypoints;
geometry_msgs::Point fileWaypoint = geometry_msgs::Point();
geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();

FSM state = FSM::ALIGN;

void addtoPath(const nav_msgs::OdometryConstPtr& msg){
    path.header = msg->header;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path.poses.push_back(pose);
    return;
}

void stopMoving(){
    cmd_vel.linear.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.linear.z = 0;
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
// double e_prev = 0;
double e_sum = 0;
double w;
double pid(double kp, double ki,double kd, double dt, double theta_ref, double theta_feedback){

    double error = theta_ref - theta_feedback;
    double e_prev = error;
    // e = error;
    
    while(error > 0.1){
        double e = error;
        e_sum = e_sum + e*dt;
        // double dedt = (e - e_prev) / dt;
        w = kp*e + ki * e_sum ;
        e_prev = e;
    }
    return w;
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

void OdomCallBack(const nav_msgs::OdometryConstPtr &msg){
    stopMoving();
    addtoPath(msg);
    updateRPY(msg->pose.pose.orientation);
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if(state == FSM::MOVE){
        cmd_vel.linear.x = 0.05;
        if(abs(x - waypoints[0].x) < TOLERANCE){
            state = FSM::ALIGN;
            ROS_WARN("\tMovement Done!!...Aligning now");
        }
        return;
    }
    if (state == FSM::ALIGN){
        double angleDiff = INITIAL_ANGLE - yaw;
        if (abs(angleDiff) > 0.003){
            if (angleDiff > 0)
                cmd_vel.angular.z = 0.3 + 0.04*angleDiff;
            else
                cmd_vel.angular.z = -0.3 + 0.04*angleDiff;
        } else {
            state = FSM::TRAJECTORY;
            ROS_WARN("\tAlignment done!!..Generating trajectory now");
        }
        return;
    }
    if(state == FSM::TRAJECTORY){
        if(current == waypoints.size()){
            return;
        }
        double nextX = waypoints[current].x;
        double nextY = waypoints[current].y;
        double dist = sqrt(pow(nextX - x,2) + pow(nextY - y,2));
        if(dist < 0.02){
            ROS_INFO("reaching waypoint number %d within a distance of %f", current, dist);
            current++;
            return;
        }
        double theta = atan2(nextY - y, nextX - x);
        cmd_vel.angular.z = getDirection(theta) * 0.9;
        // cmd_vel.angular.z = pid(kp, ki, kd, dt, theta, yaw);
        cmd_vel.linear.x = 0.8;
        return;
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "follower");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", RATE);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", RATE);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", RATE, OdomCallBack);
    ros::Rate loopRate(RATE);

    fileWaypoint.z = 0;
    std::fstream file;
    std::string line;
    file.open("/home/rajarshi/Desktop/unicycle-model-patterns/src/turtlebot3_follower/data/wp1.txt", std::ios::in);
    if (file.is_open()){
        std:: string x; std::string y;
        while(getline(file, line)){
            std::stringstream ss(line);
            ss >> x >> y;
            fileWaypoint.x = std::stod(x);
            fileWaypoint.y = std::stod(y);
            waypoints.push_back(fileWaypoint);
        }
        ROS_WARN("Number of waypoints read = %lu", waypoints.size());
        ROS_WARN("Target Point := (%f, %f)", waypoints[0].x, waypoints[0].y);
    }else{
        ROS_ERROR("Cannot open waypoints file\n");
        return EXIT_FAILURE;
    }
    while(ros::ok){
        ros::spinOnce();
        pub_cmd_vel.publish(cmd_vel);
        path_pub.publish(path);
        loopRate.sleep();
    } 

    return EXIT_SUCCESS;
}