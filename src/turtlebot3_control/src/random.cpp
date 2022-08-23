#include <iostream>
#include <cstdlib>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//! -------- CHANGEABLE PARAMETERS --------
double V;               // Velocity of the bot
double R_MIN;           // Minimum radius of the annular region
double R_MAX;           // Maximum radius of the annular region
double INITIAL_ANGLE;   // Initial angle of the bot w.r.t. X-Axis
int RATE;               // Rate at which the state of the bot will be updated
double tolerance;       // tolerance limit for binary search in switching implementatioin

//! -------- DERIVED PARAMETERS --------
double A1, B1, C1, D1;      // Coefficients of the generating function, g1(r)
double A2, B2, C2, D2;      // Coefficients of the generating function, g2(r)

//! -------- FINITE STATE MACHINE --------
enum FSM {              // Finite state machine declaration as enumeration
    ALIGN,              // Responsible for the bot's initial alignment angle
    TRAJECTORY_1,       // Move according to trajectory 1
    TRAJECTORY_2        // Move according to trajectory 2
};

//! -------- GLOBAL PARAMETERS --------
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();  // Command velocity that is published to the TurtleBot
tf::Quaternion q;           // Quaternion responsible for converting orientation of the bot into RPY
double roll;                // Roll angle of the bot
double pitch;               // Pitch angle of the bot
double yaw;                 // Yaw angle of the bot
double angleDiff = 0;       // Angular difference for initial alignment
double rPrev;               // Previous value of r
double rSwitching;          // Switching radius
FSM state = FSM::ALIGN;     // State the bot currently is in

double g1(double r){
    return (A1*r*r*r + B1*r*r + C1*r + D1);
}
double f1(double r){
    return (3*A1*r + 2*B1 + C1/r);
}

double g2(double r){
    return (A2*r*r*r + B2*r*r + C2*r + D2);
}
double f2(double r){
    return (3*A2*r + 2*B2 + C2/r);
}

// Setting the derived parameters
void setDerivedParameters(){
    INITIAL_ANGLE *= M_PI / 180;
    rPrev = R_MIN;

    //! -------- SETTING g1(r) --------
    double n = double(rand()) / double(RAND_MAX);
    if (n < 0.1){
        A1 = 0;
        B1 = 0;
        C1 = (R_MAX+R_MIN)*V/(R_MAX-R_MIN);
        D1 = (V-C1) * R_MAX;
    } else if (n < 0.4){
        double r1;
        if (n < 0.25){
            r1 = rand()/RAND_MAX * R_MIN;
        } else {
            double r1_min = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r1 = (rand()/RAND_MAX + 1) * r1_min;
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
            r1 = -1 * (rand()/RAND_MAX) * R_MAX;
            r2 = (rand()/RAND_MAX) * R_MIN;
        } else if (n < 0.8){
            r1 = (rand()/RAND_MAX) * R_MIN;
            double r2_min = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r2 = (rand()/RAND_MAX + 1) * r2_min;
        } else {
            double min_r = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r1 = (rand()/RAND_MAX + 1) * min_r;
            r2 = (rand()/RAND_MAX + 2) * min_r;
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

    //! -------- SETTING g2(r) --------
    n = double(rand()) / double(RAND_MAX);
    if (n < 0.1){
        A2 = 0;
        B2 = 0;
        C2 = -(R_MAX+R_MIN)*V/(R_MAX-R_MIN);
        D2 = - (V + C2) * R_MAX;
    } else if (n < 0.4){
        double r1;
        if (n < 0.25){
            r1 = rand()/RAND_MAX * R_MIN;
        } else {
            double r1_min = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r1 = (rand()/RAND_MAX + 1) * R_MAX;
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
            r1 = -1 * (rand()/RAND_MAX) * R_MAX;
            r2 = (rand()/RAND_MAX) * R_MIN;
        } else if (n < 0.8){
            r1 = (rand()/RAND_MAX) * R_MIN;
            double r2_min = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r2 = (rand()/RAND_MAX + 1) * r2_min;
        } else {
            double min_r = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
            r1 = (rand()/RAND_MAX + 1) * min_r;
            r2 = (rand()/RAND_MAX + 2) * min_r;
        }
        double a = V * ( R_MAX+R_MIN ) / ((R_MAX - R_MIN) * (( R_MAX*R_MAX + R_MIN*R_MIN + R_MAX*R_MIN )/3 + r1*r2 - (r1 + r2)*( R_MAX + R_MIN )/2 ));
        double b = V*R_MAX - (a/3)*(R_MAX*R_MAX*R_MAX) + (a*(r1+r2)/2)*(R_MAX*R_MAX) - a*r1*r2*R_MAX;
        A2 = -a/3;
        B2 = a * (r1+r2) / 2;
        C2 = -a*r1*r2;
        D2 = -b;
    }
    ROS_WARN("\tg2(r) = %fr^3 + %fr^2 + %fr + %f\n", A2, B2, C2, D2);

    //? -------------- implementing BINARY SEARCH -------------------

    double low = R_MIN;
    double high = R_MAX;
    double mid = (low + high)/2;
    double error;
    while(abs(high - low) >= tolerance){
        mid = (high + low) / 2;
        error = g1(mid) - g2(mid);
        if (error> 0)
            high = mid;
        else
            low = mid;
    }
    rSwitching = mid;
    ROS_WARN("\tSwitching radius: %f\n", rSwitching);
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

void odometryCallback(const nav_msgs::OdometryConstPtr& msg){
    stopMoving();

    if (state == FSM::ALIGN){
        updateRPY(msg->pose.pose.orientation);
        angleDiff = abs(INITIAL_ANGLE - yaw);
        if (angleDiff > 0.003){
            if (INITIAL_ANGLE > yaw)
                cmd_vel.angular.z = 0.1 + 0.04*angleDiff;
            else
                cmd_vel.angular.z = -0.1 - 0.04*angleDiff;
        } else {
            state = FSM::TRAJECTORY_1;
            std::cout << std::endl;
            ROS_WARN("\tAlignment Done. Moving Now..");
        }
    } else {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double r = sqrt(x*x + y*y);
        cmd_vel.linear.x = V;
        if ((r-rSwitching)*(rPrev-rSwitching) <= 0){
            ROS_WARN("SWITCHING !! RADIUS == %f\n", r);
            if (state == FSM::TRAJECTORY_1)
                state = FSM::TRAJECTORY_2;
            else if (state == FSM::TRAJECTORY_2)
                state = FSM::TRAJECTORY_1;
        }
        if (state == FSM::TRAJECTORY_1)
            cmd_vel.angular.z = f1(r);
        else if (state == FSM::TRAJECTORY_2)
            cmd_vel.angular.z = f2(r);

        rPrev = r;
    }

    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "random");
    ros::NodeHandle nh;

    if (!nh.getParam("velocity", V) || !nh.getParam("minimum_radius", R_MIN) || !nh.getParam("maximum_radius", R_MAX) || !nh.getParam("refresh_rate", RATE) || !nh.getParam("initial_angle", INITIAL_ANGLE) || !nh.getParam("tolerance", tolerance)){
        ROS_ERROR("\nCan't load params..");
        ROS_ERROR("Exiting now\n");
        return EXIT_FAILURE;
    }
    setDerivedParameters();

    ros::Publisher pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", RATE);
    ros::Subscriber sub_odom_ = nh.subscribe<nav_msgs::Odometry>("odom", RATE, odometryCallback);
    ros::Rate loopRate(RATE);

    while (ros::ok()){
        ros::spinOnce();
        pub_cmd_vel_.publish(cmd_vel);
        loopRate.sleep();
    }
    
    return EXIT_SUCCESS;
}