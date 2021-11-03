#!/usr/bin/env python3
from typing import List
from controls.msg import MotorOrientation
from controls.msg import MotorState
import rospy
import controls_constants as constants
import sys
from Moteus import Moteus

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
        The rate, in Hertz, at which readings of the Moteus
        motors for their current respective statuses are taken

    num_motors : int
        The number of motors that are controller

    id_map : { int : int }
        Maps the hardware ids given to each motor to the
        CAN bus lane which that motor communicates on

    -------
    Methods
    -------
    run_motor_orientation_loop():
        Begins an infinite loop within which the instance publishes
        on the ROS topic about the current orientation of the motors
    """

    # MARK: - Constructors -

    def __init__(self, motor_list, hw_refresh_rate: float = constants.CONTROLS_PKG_DEFAULT_HW_REFRESH_RATE):
        """Construct a new wrapper around the Moteus class

        @param motor_list: A list which maps each motor id to its
        respective CAN bus lane
        
        @param hw_refresh_rate: The rate, in Hertz, at which readings of the Moteus
        motors for their current respective statuses are taken
        """
        if len(motor_list) == 0:
            raise ValueError('The wrapper class expects at least one motor')

        self.hw_refresh_rate = hw_refresh_rate
        self.num_motors = len(motor_list)
        self.id_map = motor_list

        # The arrangement of motor ids into CAN lanes can be computed
        # by processing the mapping of motors IDs to the CAN bus lanes
        moteus_ids = []
        current_max_lane_id = -1

        # We build all of data from the mapping at once
        # instead of using list/dictionary comprehensions
        # to reduce the number of iterations through the list
        for motor_id, can_bus_lane_id in motor_list.items():
            # Determines if we need to add more lanes
            if can_bus_lane_id > current_max_lane_id:
                # Add (can_bus_lane_id - current_max_lane_id) more lanes
                moteus_ids += [[] for _ in range(current_max_lane_id, can_bus_lane_id)]

            moteus_ids[can_bus_lane_id].append(motor_id)

        # Note that the call to the initializer will block
        self.moteus_instance = mot.Moteus(moteus_ids, simulation=True)

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

    def motor_description_for_id(self, hwid: int):
        """Fetches the CAN bus lane ID corresponding to the
        motor with ID _id_

        @param hwid: The hardware ID identifying the motor whose
        descriptor should be fetched

        @return: An identifier naming a distinct CAN bus lane
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
            latest_motor_state = self.moteus_instance.getParsedResults()
            motor_states = []

            # Each 'motor_desc' is a dictionary corresponding
            # to the state of a motor
            for motor_desc in latest_motor_state:
                motor_state = MotorState()
                motor_state.position = motor_desc['POSITION']
                motor_state.velocity = motor_desc['VELOCITY']
                motor_state.torque = motor_desc['TORQUE']
                motor_states.append(motor_state)

            msg = MotorOrientation(numMotors=self.num_motors, states=motor_states)
            self.current_orientation_publisher.publish(msg)
            rate.sleep()

    def __desired_orientation_callback(self, msg: MotorOrientation):
        """A method which serves as the callback function
        of the ROS subscriber representing this instance
        as a listener to the 'desired_orientation' topic

        @param msg: A message emitted by the Intel NUC
        of type MotorOrientation describing how the motors
        should be re-oriented
        """
        if len(id_map) != msg.num_motors:
            raise ValueError('Expected a exactly one desired orientation for each motor')

        # The state of the motor at index 'msg_index' corresponds to the
        # (msg_index + 1)st motor in the dictionary mapping motor ids to
        # their CAN bus lanes
        for msg_index, motor_id in enumerate(id_map.keys()):
            motor_state = msg.states[motor_id]
            self.moteus_instance.setAttributes(motor_id, pos=motor_state.position, velocity=motor_state.velocity, torque=motor_state.torque)

    # MARK - Python Magic Methods -

    def __str__(self):
        """Outputs a formatted string describing the instance"""
        name = 'RaspberryPiMoteusWrapper\n'
        subscribed_to = 'subscribed to: ' + constants.CONTROLS_PKG_DESIRED_ORIENTATION_TOPIC + "\n"
        pub_to = 'publishing to: ' + constants.CONTROLS_PKG_CURRENT_ORIENTATION_TOPIC
        return name + subscribed_to + pub_to

# Connect to the ROS subsystem
rospy.init_node(constants.CONTROLS_PKG_RASPBERRYPI_NODE_NAME)

# Run the node
wrapper = RaspberryPiMoteusWrapper(motor_list=constants.CONTROLS_PKG_BUS_CONFIGURATION)
wrapper.run_motor_orientation_loop()