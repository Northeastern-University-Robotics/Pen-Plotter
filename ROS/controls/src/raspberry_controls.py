from typing import List
from controls.msg import MotorOrientation
from motor_descriptor import MoteusMotorDescriptor
import rospy
import moteus
import controls_constants as constants

class RaspberryPiMoteusWrapper:
    """
    Provides a ROS interface to the Moteus motor hardware

    The RaspberryPiMoteusWrapper is an adaptor which transforms
    incoming sensor data from the Moteus motor hardware into
    ROS messages (as the Moteus interface is incompatible with ROS).

    You specify the configuration of each motor that is in control
    by supplying a 'MoteusMotorDescriptor' for each of the motors

    Each instance of the RaspberryPiMoteusWrapper services a
    fixed number of motors; you cannot change the number
    (or names) of the motors that are serviced by each instance.
    Instead, create a new instance of the class and replace the
    old instance to add more motors

    NOTE: The class is intended to function as a singleton, as
    each instance serves as a publisher to the 'current_orientation' topic.

    NOTE: This class is not thread-safe. You must ensure that concurrent
    accesses are properly synchronized

    ----------
    Attributes
    ----------
    hw_refresh_rate : float
        The rate, in seconds, at which the moteus hardware is queried
        for state changes

    num_motors : int
        The number of motors that are controlled

     name_map : { str : MoteusMotorDescriptor }
        Maps the names given to each motor at initialization-time to a
        descriptor describing how that motor is configured

    id_map : { int : MoteusMotorDescriptor }
        Maps the hardware ids given to each motor to a
        descriptor describing how that motor is configured

    -------
    Methods
    -------
    run_motor_orientation_loop():
        Begins an infinite loop within which the instance publishes
        on the ROS topic about the current orientation of the motors
    """

    # MARK: - Constructors -

    def __init__(self, motor_list: List[MoteusMotorDescriptor], hw_refresh_rate: float = constants.CONTROLS_PKG_DEFAULT_HW_REFRESH_RATE):
        """Construct a new wrapper around the Moteus class

        @param motor_list: A list of descriptors describing how each motor is arranged
        
        @param hw_refresh_rate: The rate, in HZ, at which the Moteus motors
        is pinged for their current respective statuses
        """
        if len(motor_list) == 0:
            raise ValueError('The wrapper class expects at least one motor')

        self.hw_refresh_rate = hw_refresh_rate
        self.num_motors = len(motor_list)
        self.name_map = {}
        self.id_map = {}

        # The arrangement of motor ids into CAN lanes can be computed
        # by processing the descriptors one-by-one
        moteus_ids = []
        current_max_lane_id = -1

        # We build all of data from the descriptors at once
        # instead of using list/dictionary comprehensions
        # to reduce the number of iterations through the list
        for motor_desc in motor_list:
            self.name_map[motor_desc.name] = motor_desc
            self.id_map[motor_desc.hardware_id] = motor_desc
            lane_id = motor_desc.can_bus_lane_id

            # Determines if we need to add more lanes
            if lane_id > current_max_lane_id:
                # Add (lane_id - current_max_lane_id) more lanes
                moteus_ids += [[] for _ in range(current_max_lane_id, lane_id)]

            moteus_ids[lane_id] = motor_desc.hardware_id

        self.moteus_instance = moteus.Moteus(moteus_ids)

        self.desired_orientation_sub = rospy.Subscriber(
            constants.CONTROLS_PKG_DESIRED_ORIENTATION_TOPIC,
            MotorOrientation,
            callback=self.__desired_orientation_callback,
            queue_size=constants.CONTROLS_PKG_SUBSCRIBER_QUEUE_SIZE
        )

        self.current_orientation_publisher = rospy.Publisher(
            constants.CONTROLS_PKG_CURRENT_ORIENTATION_TOPIC,
            MotorOrientation,
            queue_size=constants.CONTROLS_PKG_PUBLISHER_QUEUE_SIZE
        )

    # MARK: - Methods -

    def motor_description_for_name(self, name: str):
        """Fetches the motor descriptor corresponding to the
        tag _name_ given to some motor at initialization time

        @param name: The name identifying the motor whose
        descriptor should be fetched

        @return: A MoteusMotorDescriptor which describes how the
        motor named _name_ communicates with this process
        """
        return self.name_map[name]

    def motor_description_for_id(self, hwid: int):
        """Fetches the motor descriptor corresponding to the
        motor with ID _id_

        @param hwid: The hardware ID identifying the motor whose
        descriptor should be fetched

        @return: A MoteusMotorDescriptor which describes how the
        motor with ID _id_ communicates with this process
        """
        return self.id_map[hwid]


    # MARK: - ROS Messages -

    def run_motor_orientation_loop(self):
        """Starts the infinite loop to continuously
        publish ROS notifications about the current state
        of the Moteus motors
        """
        rate = rospy.Rate(self.hw_refresh_rate)

        # Wait until the motors are ready (if they aren't already)
        self.moteus_instance.waitUntilReady()

        while (not rospy.is_shutdown()):
            latest_moteus_hardware_state = self.moteus_instance.getParsedResults()

            # Publish here

            rate.sleep()

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
            self.moteus_instance.setAttributes(motor_id, pos=pos, velocity=vel, torque=torque)

    # MARK - Python Magic Methods -

    def __str__(self):
        """Outputs a formatted string describing the instance"""
        name = 'RaspberryPiMoteusWrapper\n'
        subscribed_to = 'subscribed to: ' + RaspberryPiMoteusWrapper.SUBSCR_TOPIC_NAME + "\n"
        pub_to = 'publishing to: ' + RaspberryPiMoteusWrapper.PUB_TOPIC_NAME
        return name + subscribed_to + pub_to

if __name__ == 'main':
    motor_left = MoteusMotorDescriptor('left-motor', 1, 0)
    motor_middle = MoteusMotorDescriptor('middle-motor', 2, 1)
    motor_right = MoteusMotorDescriptor('right-motor', 3, 1)

    wrapper = RaspberryPiMoteusWrapper(
        motor_list=[motor_left, motor_middle, motor_right], 
        hw_refresh_rate=0.1
    )
    wrapper.run_motor_orientation_loop()