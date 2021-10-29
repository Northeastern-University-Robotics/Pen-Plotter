#include "ros/ros.h"
#include "../msgs/motor_orientation.msg"


class MotorControls{

private:

    /*
     *Nodehandle is the node for ros
     *current_sub subscribes to the current motor orientation
     *current_pub publishes to the raspberry pi the desired motor orientation
     */ 
    ros::NodeHandle nh;
    ros::Subscriber current_sub;
    ros::Publisher current_pub;

    /*
     * variables to contain the data of the orientation message
     */
    long numMotors;
    long numAttributes;
    std::vector<float> attributes;

    /*
     * OrientationCallback runs after the subscriber gets a message
     * desiredDirection is called to transform cartesian coordinates into motor orientation messages
     */
    void orientationCallback(const msgs::motor_orientation::ConstPtr& msg);
    float desiredDirection(msgs::motor_orientation msg, geometry_msgs::Point location);
    int main();

public:
    MotorControls(int argc, char **argv);  
};
