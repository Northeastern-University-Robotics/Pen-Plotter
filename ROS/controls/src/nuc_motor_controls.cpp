#include "nuc_motor_controls.h"
#include <cstdio>

/* Constructs a new motor controls class 
 * This class handles the calculation of transformation between cartesian and motor coordinates
 */
   
MotorControls::MotorControls(int argc, char **argv){

    /*This instantiates the nodehandles  as well as the publisher and subscruber
    *current_sub is the subscriber to current_orientation from the raspberry pi, which contains the current number of motors, and their positions
    *desired_sub is the subscriber to the desired position delta from the controls
    *desired_pub is the publisher to desired_orientation, which is sent to the pi, which contains the desired number of motors and positions
    */
    nh = ros::NodeHandle("Nuc");
    desired_sub = nh.subscribe("desired_position", 1000, &MotorControls::desiredCallback,this);
    current_sub = nh.subscribe("current_orientation", 1000, &MotorControls::orientationCallback,this);
    desired_pub = nh.advertise<controls::MotorOrientation>("desired_orientation", 1000);

    /*
     *The initial x,y, and z points (measured) 
     *input to initalize the ecoordinates
     */

    INITIAL_POINT.x = 0.5;
    INITIAL_POINT.y = 1.0;
    INITIAL_POINT.z = 0.0;

    initializeCoordinates(INITIAL_POINT);
}
/*
* desiredCallback is called when the subscriber gets the desired direction from perception/frontend
* it then calls desiredDirection to find the orientation
* It publishes the new orientation to desired_orientation, and sends the message
* 
*/
void MotorControls::desiredCallback(const geometry_msgs::Point::ConstPtr& location){
    if (ros::ok()){
        desired_pub.publish(desiredDirection(location));

    }
}

/*
* orientationCallback is called when the subscriber gets the current orientation 
* it then updates the motor orientation stored in the class
* 
*/
void MotorControls::orientationCallback(const controls::MotorOrientation::ConstPtr& msg){
    if(ros::ok()){
        motor_orientation.numMotors = msg->numMotors;
        motor_orientation.states = msg->states;
    }
}

/*
*This function is called on startup, and it gets the _0 variables an initializes the inital position
*/
void MotorControls::initializeCoordinates(geometry_msgs::Point& initial){
    loc = initial;
    r_0[0] = sqrt(pow(initial.y,2)+pow(initial.x,2) - pow(rm,2));
    r_0[1] = sqrt(pow(x2 - initial.x,2)+pow(initial.x,2) - pow(rm,2));
    angle_0[0] = atan(initial.y/(initial.x)) + asin(sqrt(pow(initial.x,2)+pow(initial.x,2) - pow(rm,2))/sqrt(pow(initial.x,2)+pow(initial.x,2)));
    angle_0[1] = atan(initial.y/(x2-initial.x)) + asin(sqrt(pow(initial.x,2)+pow(initial.x,2) - pow(rm,2))/sqrt(pow(initial.x,2)+pow(initial.x,2)));
    std::cout<<"finished initialization \n";
}

/*
 * Inputs the desired direction in cartesian coordinates 
 * Outputs the desired motor orientation in relation to the motors to the pi
 * stores point in a log and calculates motor angles to send
 * 
 */
controls::MotorOrientation MotorControls::desiredDirection(const geometry_msgs::Point::ConstPtr& location){
    
    /*
     * Updates the location's coordinates and puts it into a log
     * 
     */
    loc.x += location->x;
    loc.y += location->y;
    loc.z += location->z;
    point_log.push_back(loc);

    /*
     * Actual calculation of motor angles
     */
    angle_m[0] =(sqrt(pow(loc.x,2)+pow(loc.x,2) - pow(rm,2))- r_0[0])/rm 
            - atan(loc.y/loc.x) 
            + asin(sqrt(pow(loc.x,2)+pow(loc.x,2) - pow(rm,2))/
                    sqrt(pow(loc.x,2)+pow(loc.x,2))) + angle_0[0];
    angle_m[1]=(sqrt(pow(loc.x,2)+pow(loc.x,2) - pow(rm,2))-r_0[1])/rm 
            - atan(loc.y/loc.x) 
            + asin(sqrt(pow(loc.x,2)+pow(loc.x,2) - pow(rm,2))/
                    sqrt(pow(loc.x,2)+pow(loc.x,2))) + angle_0[1];

    for(int i = 0; i< motor_orientation.numMotors; i++){
        motor_orientation.states[i].position = angle_m[i];
    }


    return motor_orientation;

}

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

/*
* In main, the nuc should be initialized, and then just listen for signals with spin 
* 
*/
int main(int argc, char **argv){
    std::cout<<"Start Nuc program \n";
    ros::init(argc, argv, "Nuc");
    MotorControls nuc_controller = MotorControls(argc, argv);
    ros::spin();

    // this should either be just ros::spin();, or ros::spineOnce() in the while loop
    return 0;
}






