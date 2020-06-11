/*
    More complete example: 
        
        The attempt of this code is to give the robot coordinate 
        destination (x, y) and the robot should go to that 
        new pose.   
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

using namespace std;
const double PI = 3.14159265359;

ros::Publisher vel_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_current_pose;


double setDesireOrientation(double desired_angle_radians);
void poseCallback(const turtlesim::Pose::ConstPtr &pose_message);
void rotate(double angular_speed, double angle, bool clockwise);
double degree2radians(double angle_in_degree);
void move(double speed, double distance, bool isForward);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
double getDistance(double x1, double y1, double x2, double y2);


int main(int argc, char **argv){

    ros::init(argc, argv, "robot_goal");
    ros::NodeHandle nh;

    vel_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pose_subscriber = nh.subscribe("/turtle1/pose", 10, poseCallback);
    
    ros::Rate loop_rate(0.5);

    turtlesim::Pose goal_pose;

    goal_pose.x = 10;
    goal_pose.y = 2;
    goal_pose.theta = 45;

    moveGoal(goal_pose, 0.01);
    loop_rate.sleep();

    ros::spin();
    
    return 0;
}


void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){

    geometry_msgs::Twist vel_to_goal_msg;

    ros::Rate loop_rate(10);

    do
    {
        vel_to_goal_msg.linear.x = 1.5 * getDistance(turtlesim_current_pose.x, turtlesim_current_pose.y, 
                                                     goal_pose.x, goal_pose.y);
        vel_to_goal_msg.linear.y = 0;
        vel_to_goal_msg.linear.z = 0;

        vel_to_goal_msg.angular.x = 0;
        vel_to_goal_msg.angular.y = 0;
        vel_to_goal_msg.angular.z = 4*(atan2(goal_pose.y - turtlesim_current_pose.y, goal_pose.x-turtlesim_current_pose.x) - turtlesim_current_pose.theta);

        vel_publisher.publish(vel_to_goal_msg);
       
        ros::spinOnce();
        loop_rate.sleep();

    } while (getDistance(turtlesim_current_pose.x, turtlesim_current_pose.y, goal_pose.x, goal_pose.y)> distance_tolerance);
    
    cout << "Goal has been reached" << endl;

    vel_to_goal_msg.linear.x = 0;
    vel_to_goal_msg.angular.z = 0;
    vel_publisher.publish(vel_to_goal_msg);
}

double getDistance(double x1, double y1, double x2, double y2){
    return sqrt(pow((x1-x2), 2) + pow((y1-y2), 2) );
}


void poseCallback(const turtlesim::Pose::ConstPtr &pose_message){
    turtlesim_current_pose.x = pose_message->x;
    turtlesim_current_pose.y = pose_message->y;
    turtlesim_current_pose.theta = pose_message->theta;

}

double setDesireOrientation(double desired_angle_radians){

    double relative_angle_radians = desired_angle_radians - turtlesim_current_pose.theta;
    bool clockwise = ((relative_angle_radians < 0)?true:false);
    
    rotate(abs(relative_angle_radians), abs(relative_angle_radians), clockwise);

}

void rotate(double angular_speed, double relative_angle, bool clockwise){
    
    geometry_msgs::Twist rotation_msg;

    rotation_msg.linear.x = 0;
    rotation_msg.linear.y = 0;
    rotation_msg.linear.z = 0;

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
    
    do{
        vel_publisher.publish(rotation_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    }while(current_angle < relative_angle);

    rotation_msg.angular.z = 0.0;
    vel_publisher.publish(rotation_msg);

}


void move(double speed, double distance, bool isForward){

    geometry_msgs::Twist vel_msg;

    if(isForward){
        vel_msg.linear.x = abs(speed);
    }
    else{
        vel_msg.linear.x = -abs(speed);
    }
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    /*
        t0: current time 
        loop
            publish velocity
            estimate the current_distance = speed * (t1-t0)
            current_distance_moved_by_robot <= distance
    */

   double t0 = ros::Time::now().toSec();
   double current_distance = 0;
   ros::Rate loop_rate(10);

    do{
        vel_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();

    }while(current_distance < distance);

    // once out of the loop set vel to 0, otherwise it will keep publishing
    vel_msg.linear.x = 0.0;
    vel_publisher.publish(vel_msg);
}

double degree2radians(double angle_in_degree){
    return angle_in_degree * PI / 180;
}