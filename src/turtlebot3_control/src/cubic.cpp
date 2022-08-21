#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<tf/tf.h>
#include<cstdlib>
#include<time.h>
#include<iostream>

//! -------- CHANGEABLE PARAMETERS --------
double V;               // Velocity of the bot
double R_MIN;           // Minimum radius of the annular region
double R_MAX;           // Maximum radius of the annular region
double INITIAL_ANGLE;   // Initial angle of the bot w.r.t. X-Axis
int RATE;               // Rate at which the state of the bot will be updated
double r1;
double r2;
//! -------- DERIVED PARAMETERS --------
double a;               // Slope of the linear generating function
double b;               // y-intercept of the linear generatung function

//! -------- FINITE STATE MACHINE --------
enum FSM {              // Finite state machine declaration as enumeration
    ALIGN,              // Responsible for the bot's initial alignment angle
    MOVE                // Responsible for moving the bot according to the control law
};

void setDerivedParameters(double n){
    INITIAL_ANGLE *= M_PI / 180;
    double r1_min;
    double r2_min;
    double r_min;

    if(n < 0.2){
        r1 = -1 * ( rand() / RAND_MAX )*R_MAX;
        r2 = ( rand() / RAND_MAX )*R_MIN;
    }else if(n < 0.6){
        r1 = ( rand() / RAND_MAX )*R_MIN;
        r2_min = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
        r2 = (rand() / RAND_MAX + 1)* r2_min;
    }else{
        r_min = ((R_MIN + R_MAX)*(R_MAX + R_MIN))/4*R_MIN;
        r1 = ( rand() / RAND_MAX + 1)*r_min;
        r2 = ( rand() / RAND_MAX + 2)*r_min;
    }
    a = V * ( R_MAX+R_MIN ) / ((R_MAX - R_MIN) * (( R_MAX*R_MAX + R_MIN*R_MIN + R_MAX*r_min )/3 + r1*r2 - (r1 + r2)*( R_MAX + R_MIN )/2 ));
    b = V*R_MAX - (a/3)*(R_MAX*R_MAX*R_MAX) + (a*(r1+r2)/2)*(R_MAX*R_MAX) - a*r1*r2*R_MAX;

    std::cout << "--------- Displaying Parameters --------" << std::endl;
    std::cout << "R_MIN : " << R_MIN << std::endl;
    std::cout << "R_MAX : " << R_MAX << std::endl;
    std::cout << "INITIAL_ANGLE : " << INITIAL_ANGLE << std::endl;
    std::cout << "a : " << a << std::endl;
    std::cout << "b : " << b << std::endl;
    std::cout << "r1 : " << r1 << std::endl;
    std::cout << "r2 : " << r2 << std::endl;
    std::cout << "r2_min : " << r2_min << std::endl;
    std::cout << "r_min : " << r_min << std::endl;
    std::cout << "velocity : " << V << std::endl;

}

double f(double r){
    return a*r*r + a*r*(r1 + r2) + a*r1*r2;
}

//! -------- GLOBAL PARAMETERS --------
geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();  // Command velocity that is published to the TurtleBot
tf::Quaternion q;           // Quaternion responsible for converting orientation of the bot into RPY
double roll;                // Roll angle of the bot
double pitch;               // Pitch angle of the bot
double yaw;                 // Yaw angle of the bot
double angleDiff = 0;       // Angular difference for initial alignment
FSM state = FSM::ALIGN;     // State the bot currently is in

void stopMoving(){
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.x = 0;
    return;
}
// get roll, pitch, yaw parameters
void updateRPY(const geometry_msgs::Quaternion &orientation){
    q.setW(orientation.w);
    q.setX(orientation.x);
    q.setY(orientation.y);
    q.setZ(orientation.z);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return;
}
// calculating the angular velocity of robot which is passed as an input to the controller
double calcAlphaDot(const geometry_msgs::Point &position){
    double r = sqrt(position.x*position.x + position.y*position.y);
    r = std::min(4.0, std::max(2.0, r));
    // std::cout << f(r) << std::endl;
    return f(r);
}
// callback function for subscriber
void odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    stopMoving();

    if(state == FSM::ALIGN){
        updateRPY(msg-> pose.pose.orientation);
        angleDiff = abs(INITIAL_ANGLE - yaw);
        if (angleDiff > 0.003){
            if (INITIAL_ANGLE > yaw)
                cmd_vel.angular.z = 0.1 + 0.04*angleDiff;
            else
                cmd_vel.angular.z = -0.1 - 0.04*angleDiff;
        }else{
            state = FSM::MOVE;
            std::cout << "Alignment Done!" << std::endl;
            std::cout << "Moving turtlebot now!" << std::endl;
        }
    }else if(state == FSM::MOVE){
        cmd_vel.linear.x = V;
        cmd_vel.angular.z = calcAlphaDot(msg->pose.pose.position);
    }

    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "quadratic");
    ros::NodeHandle nh;

    if (!nh.getParam("velocity", V) || !nh.getParam("minimum_radius", R_MIN) || !nh.getParam("maximum_radius", R_MAX) || !nh.getParam("refresh_rate", RATE) || !nh.getParam("initial_angle", INITIAL_ANGLE)){
        ROS_ERROR("\nCan't load params..");
        ROS_ERROR("Exiting now\n");
        return EXIT_FAILURE;
    }
    srand( (unsigned)time( NULL ) );
    double n = rand()/RAND_MAX;

    setDerivedParameters(n);

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
