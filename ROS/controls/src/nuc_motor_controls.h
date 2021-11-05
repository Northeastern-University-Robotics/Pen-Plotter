#include "ros/ros.h"
#include "../msgs/MotorOrientation.msg"
#include "../msgs/MotorState.msg"
#include <vector>;

class MotorControls{

private:

    /*
     *@param pi_nh is the node for ros that interfaces with the pi
     *@param desired_nh is the node for ros that interfaces with perceptrion
     *@param current_sub subscribes to the current motor orientation from the pi
     *@param desired_sub subscribes to the desired motor orientation from the publishers
     *@param current_pub publishes to the raspberry pi the desired motor orientation
     */ 
    ros::NodeHandle pi_nh;
    ros::NodeHandle desired_nh;
    ros::Subscriber current_sub;
    ros::Subscriber desired_sub;
    ros::Publisher desired_pub;

    /*
     * variables to contain the data of the orientation message
     * @param motor_orientation takes the motor orientation message
     * @param loc takes in the xy position
     */

    //std::vector<geometry_msgs::Point> point_log;

    msgs::MotorOrientation motor_orientation;
 //   long numMotors;
 //   MotorState[] states;
    geometry_msgs::Point loc;

    /*
     * variables for the calculatrion of the position
     * @param x2 is the distance between the motors
     * @param rm is the motor radius
     * @param r_0 are the initial motor coordinates
     * @param angle_0 are the original angles
     * 
     */
    
    float x2 = 1;
    float rm = .1;
    float r_0[2];
    float angle_0[2];
    float angle_m[2];
    /*
     * OrientationCallback runs after the subscriber gets a message
     * desiredDirection is called to transform cartesian coordinates into motor orientation messages
     */
    void orientationCallback(const msgs::motor_orientation::ConstPtr& msg);
    void desiredCallback(const msgs::motor_orientation::ConstPtr& msg));
    float desiredDirection(msgs::motor_orientation msg, geometry_msgs::Point location);
    int main();

public:
    MotorControls(int argc, char **argv);  
};
