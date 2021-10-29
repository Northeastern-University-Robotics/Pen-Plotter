#include "ros/ros.h"
#include "../msgs/motor_orientation.msg"
#include "nuc_motor_controls.h"
#include <vector>
#include <cmath>

class MotorControls{

/* Constructs a new motor controls class 
 * This class handles the calculation of transformation between cartesian and motor coordinates
 */
   
    MotorControls(int argc, char **argv){

        ros::init(argc, argv, "Nuc");

        /*This instantiates the nodehandle class as well as the publisher and subscruber
         *current_sub is the subscriber to current_orientation from the raspberry pi, which contains the current number of motors, and their positions
         *desired_pub is the publisher to desired_orientation, which is sent to the pi, which contains the desired number of motors and positions
         */
        this->nh = ros::NodeHandle();
        this->current_sub = this ->nh.subscribe("current_orientation", 1000, &nuc_motor_controls::orientationCallback, this);
        this->desired_pub = this->nh.advertise<msgs::motor_orientation>("desired_orientation", 1000);

        ros::Rate loop_rate(10);



    }
    /*
     * orientationCallback is called when the subscriber gets the current orientation 
     * it then calls desiredDirection to find the desired direction
     * It publishes the new direction to desired_orientation, and sends the message
     * 
     */
    void orientationCallback(const msgs::motor_orientation::ConstPtr& msg){
        if ros::ok(){
            this->desired_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    /*
     * WORK IN PROGRESS
     * Should input the desired direction in cartesian coordinates and output the desired motor orientation in relation to the motors
     * 
     */
    float desiredDirection(msgs::motor_orientation msg, geometry_msgs::Point location){
        /*
        geometry_msgs::Point loc = location;
        float rm = .1;
        float r10 = 1;
        float r20 = 1;
        float angle10 = 45;
        float angle20 = 45;
        float x2 = 1;
        float r1 = Sqrt(pow(loc.x,2))+pow(loc.x,2) - pow(rm,2)); 
        float r2 = Sqrt(pow(loc.x - x2,2))+pow(loc.x,2) - pow(rm,2));
        float r1p = Sqrt(pow(loc.x,2))+pow(loc.x,2)); 
        float r2p = Sqrt(pow(loc.x - x2,2))+pow(loc.x,2));
        float angle1 = atan(loc.y/loc.x);
        float angle2 = atan(loc.y/(loc.x-x2));
        float angle1p = angle1 + asin(r1/r1p);
        float angle2p = angle2 - asin(r2/r2p);
        float posm1 = (r1-r10)/rm - angle1p + angle10;
        desired_msg.attributes
        */
       return 0.0;
    }

    /*
     * In main, the nuc should just listen for signals with spin
     * 
     */
    int main(){
        while(ros::ok()){
            
           ros::spin();
        }
        return 0;
    }
    





}