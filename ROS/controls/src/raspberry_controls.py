from typing import List
from controls.msg import MotorOrientation
#import moteus
import rospy
import datetime

class RaspberryPiMoteusWrapper:
    """
    Provides a ROS interface to the Moteus motor hardware
    
    The RaspberryPiMoteusWrapper is an adaptor which transforms
    incoming sensor data from the Moteus motor hardware into
    ROS messages (the moteus interface is incompatible with ROS).
    Details about the actual moteus motors is mostly irrelvant
    to the interface provided by the class; clients need only
    label particular motors and identify how often the hardware
    should be queried for state changes.

    Each instance of the RaspberryPiMoteusWrapper services a 
    fixed number of moteus motors; you cannot change the number
    (or names) of the motors that are serviced by each instance.
    Instead, create a new instance of the class and replace the 
    old instance to add more motors

    NOTE: The class is intended to function as a singleton, as 
    each instance serves as a publisher to the ?INSERT TOPIC NAME HERE?
    topic.

    ...

    Attributes
    ----------
    hw_refresh_rate : float
        The rate, in Hertz, at which the moteus hardware is queried 
        for state changes
    num_motors : int
        The number of motors that are controlled
    id_map : { str : int }
        Maps the names given to each motor to their representation
        by the Moteus interface wrapper (their id)

    desired_orientation_sub: rospy.Subscriber
        A subscriber which listens to outgoing messages from
        the Intel NUC

    current_orientation_publisher: rosdpy.Publisher
        A publisher which periodically posts the 
        status of all the Moteus motors this instance
        keeps tracks of

    Methods
    -------
    run_motor_orientation_loop():
        Begins an infinite loop within which the instance publishes
        on the ROS topic about the current orientation of the motors
    """

    # The maximum number of ROS messages that are buffered
    # by both the publisher and subscibers created by this instance
    QUEUE_SIZE = 10
    
    # The name of the topic the nodes created by this class
    # subscribe to
    SUBSCR_TOPIC_NAME = 'desired_orientation'

    # The name of the topic the nodes created by this class
    # publish to
    PUB_TOPIC_NAME = 'current_orientation'

    # MARK: - Constructors -

    def __init__(self, names: List[str], hw_refresh_rate: float = 0.016): 
        """Construct a new wrapper around the Moteus class
        which manages the motor hardware communication 

        @param names: A list of names identifying each motor that
        is controlled by this instance. Each motor is assigned a
        unique id which serves to identify the motor from the 
        perspective of Moteus. The ids are strictly increasing
        asnd begin at 0

        @param hw_refresh_rate: The rate, in HZ, at which the moteus hardward
        is pinged for its current status
        """

        if len(names) == 0:
            raise ValueError('The wrapper class expects at least one motor')

        num_motors = len(names)
        ids = [i for i in range(num_motors)]

        self.hw_refresh_rate = hw_refresh_rate
        self.num_motors = num_motors
        self.id_map = { names[i] : i for i in range(num_motors) }
        #self.moteus_instance = moteus.Moteus(ids)

        self.desired_orientation_sub = rospy.Subscriber(
            'desired_orientation', 
            MotorOrientation, 
            callback=self.__desired_orientation_callback,
            queue_size=RaspberryPiMoteusWrapper.QUEUE_SIZE
        )

        self.current_orientation_publisher = rospy.Publisher(
            'current_orientation', 
            MotorOrientation, 
            queue_size=RaspberryPiMoteusWrapper.QUEUE_SIZE
        )

    # MARK: - Methods -

    def __get_motor_id(self, name):
        """Fetches the motor id assigned by moteus
        (implicitly assigned at initialization-time
        by this class) that corresponds to the human-readable
        string name _name_

        @param name: The name identifying the motor whose id
        should be fetched

        @return: An integer identifier that Moteus uses to
        identify the motor with this name
        """
        return self.id_map[name]

    # MARK: - ROS Messages -

    def __desired_orientation_callback(self, msg: MotorOrientation):
        """A method which serves as the callback function
        of the ROS subscriber representing this instance 
        as a listener to the 'desired_orientation' topic

        @param msg: A message emitted by the Intel NUC
        of type MotorOrientation describing how the motors 
        should be re-oriented
        """
        for motor_id in range(msg.numMotors):
            pos = msg.attributes[3 * motor_id]
            vel = msg.attributes[3 * motor_id + 1]
            torque = msg.attributes[3 * motor_id + 2]
            #self.moteus_instance.setAttributes(motor_id, pos=pos, velocity=vel, torque=torque)
    
    def run_motor_orientation_loop(self):
        """Starts the infinite loop to continuously
        publish ROS notifications about the current state
        of the Moteus motors
        """
        rate = rospy.Rate(self.hw_refresh_rate)

        while (not rospy.is_shutdown()):
            latest_moteus_hardware_state = self.moteus_instance.getParsedResults()
            rate.sleep()
            

    # MARK - Python Magic Methods -

    def __str__(self):
        """Outputs a formatted string describing the instance"""
        name = 'RaspberryPiMoteusWrapper\n'
        subscribed_to = 'subscribed to: ' + RaspberryPiMoteusWrapper.SUBSCR_TOPIC_NAME + "\n"
        pub_to = 'publishing to: ' + RaspberryPiMoteusWrapper.PUB_TOPIC_NAME
        return name + subscribed_to + pub_to
        

if __name__ == 'main':
    wrapper = RaspberryPiMoteusWrapper(names=['left-motor'], hw_refresh_rate=0.1)
    wrapper.run_motor_orientation_loop()