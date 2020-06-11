#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


ros::Publisher vel_cmd_publisher;
/*
    The turtle simulation is moved using the topic 
        /turtle1/cmd_vel
    
    The velocity/movement command is send via the message
        /geometry_msgs/Twist 
*/
void move(double speed, double distance, bool isForward);

int main(int argc, char **argv){

    double speed;
    double distance;
    bool isForward;

    // 1. Initialize the node to 'turtle_move_fb' 
    // f : forward ; b: backwards
    ros::init(argc, argv, "turtle_move_fb");

    // 2. create a node handle 
    ros::NodeHandle nh;

    // 3. Create a global publisher object (outside main() )
    //    and publish to the topic /turtle1/cmd_vel

    vel_cmd_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
   

    std::cout << "enter speed: ";
    std::cin >> speed;
    std::cout << "enter distance (0 - 5): ";
    std::cin >> distance;
    std::cout << "forward or backward (1, 0): ";
    std::cin >> isForward;

    move(speed, distance, isForward);
    
    return 0;

}

/*
    Distance traveled is calculated by the following formula:
        distance = speed * time
    where time is:
        time = (t1 - t0)
*/
void move(double speed, double distance, bool isForward){

    geometry_msgs::Twist vel_cmd_msg;

    if(isForward){
        vel_cmd_msg.linear.x = abs(speed);
    }
    else{
        vel_cmd_msg.linear.x = -abs(speed);
    }
    
    // Everything else is set to 0 because we are
    // only moving forward or backward and not rotation
    vel_cmd_msg.linear.y = 0.0;
    vel_cmd_msg.linear.z = 0.0;
    
    vel_cmd_msg.angular.x = 0;
    vel_cmd_msg.angular.y = 0;
    vel_cmd_msg.angular.z = 0;

    double t0 = ros::Time::now().toSec();
    double current_distance  = 0;          //current distance traveled by the robot
    ros::Rate loop_rate(10);

    // we need to continously advertize until the robot
    // reached the desired distance.

    do{

        vel_cmd_publisher.publish(vel_cmd_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1 - t0);

        ros::spinOnce();
        loop_rate.sleep();

    }while(current_distance < distance);

    // once the loop is over we need to set the vel in 
    // x-direction back to 0 and send the command to the
    // robot

    vel_cmd_msg.linear.x = 0.0;
    vel_cmd_publisher.publish(vel_cmd_msg);

}