/*
    This program will make the turtle robot rotate a given angle:
        - Counter-clockwise
        - Clockwise
    
    Since we need to send a command to the robot, the program is a
    publisher to a topic via the geometry_msgs/Twist message type.
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;

const double PI = 3.14159265359;

// Global publisher object
ros::Publisher rotate_cmd_publisher;


void rotate(double angular_speed, double desired_angle, bool clockwise);
double degree2radians(double angle_in_degree);


int main(int argc, char **argv){

    double angular_speed;
    double angle;
    bool clockwise;

    // 1. Initialize the node
    ros::init(argc, argv, "turtle_rotate_left_right");

    // 2. ROS handle object 
    ros::NodeHandle nh;

    // 3. Publish 
    rotate_cmd_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    cout << "enter angular velocity: ";
    cin >> angular_speed;
    cout << "enter desired angle: ";
    cin >> angle;
    cout << "Clockwise? (0/1): ";
    cin >> clockwise;

    // 4. create and test rotate() function 
    rotate(degree2radians(angular_speed), degree2radians(angle), clockwise);

    return 0;
}

void rotate(double angular_speed, double desired_angle, bool clockwise){
    
    // 1. Create a Twist message object
    geometry_msgs::Twist rotation_msg;

    // 2. Since we are rotating no linear movement so set to 0
    rotation_msg.linear.x = 0;
    rotation_msg.linear.y = 0;
    rotation_msg.linear.z = 0;

    // 3. we are rotating along the Z axis so set x and y axis to 0
    rotation_msg.angular.x = 0;
    rotation_msg.angular.y = 0;


    if(clockwise){
        rotation_msg.angular.z = -abs(angular_speed);
    }
    else{
        rotation_msg.angular.z = abs(angular_speed);
    }

    double current_angle = 0.0;
    double t0 = ros::Time::now().toSec();
    ros::Rate loop_rate(10);
    
    // loop to constantly publish until the desired angle is
    // reached
    do{
        rotate_cmd_publisher.publish(rotation_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    }while(current_angle < desired_angle);

    // set angular z axis to 0 and publish so that the 
    // robot stops rotating.
    rotation_msg.angular.z = 0.0;
    rotate_cmd_publisher.publish(rotation_msg);
}


// simple function to convert degree to radians
double degree2radians(double angle_in_degree){
    return angle_in_degree * PI / 180;
}