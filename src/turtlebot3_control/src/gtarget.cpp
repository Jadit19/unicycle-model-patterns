#include <iostream>
#include <cstdlib>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

//! -------- CHANGEABLE PARAMETERS --------
const int RATE = 30;
const double V = 0.01;
const double R_MIN = 1.0;
const double R_MAX = 1.5;
const double TOLERANCE = 0.001;
double INITIAL_ANGLE = -M_PI_2;
std::vector<std::pair<double, double>> targets;

//! -------- DERIVED PARAMETERS --------
double A1, B1, C1, D1;      // Coefficients of the generating function, g1(r)
double A2, B2, C2, D2;      // Coefficients of the generating function, g2(r)

//! -------- FINITE STATE MACHINE --------
enum FSM {              // Finite state machine declaration as enumeration
    MOVE,               // Move the bot to the required location
    ALIGN,              // Responsible for the bot's initial alignment angle
    TRAJECTORY
};

double r;
double x; double y;

//! -------- GLOBAL PARAMETERS --------
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();          // Command velocity that is published to the TurtleBot
nav_msgs::Path path = nav_msgs::Path();                         // Path that the turtlebot follows, used in RViz
geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped(); // Pose of the bot, used in RViz
tf::Quaternion q;           // Quaternion responsible for converting orientation of the bot into RPY
bool availableToSwitch = 1; // Determines is the bot is ready to switch
double roll;                // Roll angle of the bot
double pitch;               // Pitch angle of the bot
double yaw;                 // Yaw angle of the bot
double angleDiff = 0;       // Angular difference for initial alignment
double rPrev;               // Previous value of r
double rSwitching;          // Switching radius
int count = 0;              // No. of times the bot has crossed the switching radius
FSM state = FSM::MOVE;      // State the bot currently is in

double g(double r){
    return (A1*r*r*r + B1*r*r + C1*r + D1);
}
double f(double r){
    return (3*A1*r + 2*B1 + C1/r);
}



// Generating random numbers between 0 and 1
double genRand(){
    return (double(rand()) / double(RAND_MAX));
}

void addToPath(const nav_msgs::OdometryConstPtr& msg){
    path.header = msg->header;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path.poses.push_back(pose);
    return;
}
// Setting the derived parameters
void setDerivedParameters(){
    srand((unsigned int) time(NULL));
    INITIAL_ANGLE *= M_PI/180;
    rPrev = R_MIN;

    // -------- SETTING g1(r) --------
    double n = genRand();
    if (n < 0.1){
        A1 = 0;
        B1 = 0;
        C1 = (R_MAX+R_MIN)*V/(R_MAX-R_MIN);
        D1 = (V-C1) * R_MAX;
    } else if (n < 0.4){
        double r1;
        if (n < 0.25){
            r1 = genRand() * R_MIN;
        } else {
            double r1_min = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r1 = (genRand() + 1) * r1_min;
        }
        double a = V * (R_MAX+R_MIN) / ((R_MAX-R_MIN) * ((R_MAX+R_MIN)/2 - r1));
        double b = V*R_MAX + a*r1*R_MAX - a*R_MAX*R_MAX/2;
        A1 = 0;
        B1 = a/2;
        C1 = -a*r1;
        D1 = b;
    } else {
        double r1, r2;
        if (n < 0.6){
            r1 = -1 * (genRand()) * R_MAX;
            r2 = (genRand()) * R_MIN;
        } else if (n < 0.8){
            r1 = (genRand()) * R_MIN;
            double r2_min = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r2 = (genRand() + 1) * r2_min;
        } else {
            double min_r = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r1 = (genRand() + 1) * min_r;
            r2 = (genRand() + 2) * min_r;
        }
        double a = V * ( R_MAX+R_MIN ) / ((R_MAX - R_MIN) * (( R_MAX*R_MAX + R_MIN*R_MIN + R_MAX*R_MIN )/3 + r1*r2 - (r1 + r2)*( R_MAX + R_MIN )/2 ));
        double b = V*R_MAX - (a/3)*(R_MAX*R_MAX*R_MAX) + (a*(r1+r2)/2)*(R_MAX*R_MAX) - a*r1*r2*R_MAX;
        A1 = a/3;
        B1 = -a * (r1+r2) / 2;
        C1 = a*r1*r2;
        D1 = b;
    }
    std::cout << std::endl;
    ROS_WARN("\tg1(r) = %fr^3 + %fr^2 + %fr + %f", A1, B1, C1, D1);

    // -------- SETTING g2(r) --------
    n = genRand();
    if (n < 0.1){
        A2 = 0;
        B2 = 0;
        C2 = -(R_MAX+R_MIN)*V/(R_MAX-R_MIN);
        D2 = - (V + C2) * R_MAX;
    } else if (n < 0.4){
        double r1;
        if (n < 0.25){
            r1 = genRand() * R_MIN;
        } else {
            double r1_min = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r1 = (genRand() + 1) * R_MAX;
        }
        double a = V * (R_MAX+R_MIN) / ((R_MAX-R_MIN) * ((R_MAX+R_MIN)/2 - r1));
        double b = V*R_MAX + a*r1*R_MAX - a*R_MAX*R_MAX/2;
        A2 = 0;
        B2 = -a/2;
        C2 = a*r1;
        D2 = -b;
    } else {
        double r1, r2;
        if (n < 0.6){
            r1 = -1 * (genRand()) * R_MAX;
            r2 = (genRand()) * R_MIN;
        } else if (n < 0.8){
            r1 = (genRand()) * R_MIN;
            double r2_min = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r2 = (genRand() + 1) * r2_min;
        } else {
            double min_r = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r1 = (genRand() + 1) * min_r;
            r2 = (genRand() + 2) * min_r;
        }
        double a = V * ( R_MAX+R_MIN ) / ((R_MAX - R_MIN) * (( R_MAX*R_MAX + R_MIN*R_MIN + R_MAX*R_MIN )/3 + r1*r2 - (r1 + r2)*( R_MAX + R_MIN )/2 ));
        double b = V*R_MAX - (a/3)*(R_MAX*R_MAX*R_MAX) + (a*(r1+r2)/2)*(R_MAX*R_MAX) - a*r1*r2*R_MAX;
        A2 = -a/3;
        B2 = a * (r1+r2) / 2;
        C2 = -a*r1*r2;
        D2 = -b;
    }
    ROS_WARN("\tg2(r) = %fr^3 + %fr^2 + %fr + %f\n", A2, B2, C2, D2);

}

// Stop the movement of the bot
void stopMoving(){
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    return;
}

// Calculating the roll, pitch and yaw of the bot
void updateRPY(const geometry_msgs::Quaternion& orientation){
    q.setW(orientation.w);
    q.setX(orientation.x);
    q.setY(orientation.y);
    q.setZ(orientation.z);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg){
    stopMoving();
    addToPath(msg);
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    if (state == FSM::MOVE){
        cmd_vel.linear.x = 0.05;
        if (abs(targets[0].first+R_MAX-x) < TOLERANCE){
            state = FSM::ALIGN;
            ROS_WARN("\tMovement done. Aligning now");
        }
        return;
    }
    
    if (state == FSM::ALIGN){
        updateRPY(msg->pose.pose.orientation);
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
    
    cmd_vel.linear.x = V;
    r = 1e8;
    for (std::pair<double, double> target: targets){
        r = std::min(r, sqrt(pow(target.first-x,2) + pow(target.second-y,2)));
    }
    cmd_vel.angular.z = f(r);
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pattern");
    ros::NodeHandle nh;
    setDerivedParameters();

    ros::Publisher pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", RATE);
    ros::Publisher pub_path_ = nh.advertise<nav_msgs::Path>("path", RATE);
    ros::Subscriber sub_odom_ = nh.subscribe<nav_msgs::Odometry>("odom", RATE, odomCallback);
    ros::Rate loopRate(RATE);

    targets.push_back(std::make_pair(1, 0));
    targets.push_back(std::make_pair(0, -1));
    targets.push_back(std::make_pair(-1, 0));
    targets.push_back(std::make_pair(0, 1));

    while (ros::ok()){
        ros::spinOnce();
        pub_cmd_vel_.publish(cmd_vel);
        pub_path_.publish(path);
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}