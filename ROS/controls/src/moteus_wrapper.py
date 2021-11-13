#!/usr/bin/env python3
import rospy
import sys
import signal

from typing import List
from controls.msg import CurrentMotorOrientation
from controls.msg import DesiredMotorOrientation
from controls.msg import MotorState
from Motors.Moteus import Moteus
import controls_constants as constants

class RaspberryPiMoteusWrapper:
    """
    Provides a ROS interface to the Moteus motor hardware

    The RaspberryPiMoteusWrapper is an adaptor which transforms
    incoming sensor data from the Moteus motor hardware into
    ROS messages (as the Moteus interface is incompatible with ROS).
    You describe the configuration of the motors by supplying a
    map which associates each motor (each distinguished by a unique
    id) to a CAN bus lane on which that motor is communicating.

    Each instance of the RaspberryPiMoteusWrapper services a
    fixed number of motors; you cannot change the number (or identifiers)
    of the motors that are serviced after you instantiate the class.
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

    shutdown_moteus():
        Stops the execution of the node and performs any necessary
        resource cleanup and/or hardware cleanup
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
        self.moteus_instance = Moteus(moteus_ids, simulation=True)

        self.desired_orientation_sub = rospy.Subscriber(
            constants.CONTROLS_PKG_DESIRED_ORIENTATION_TOPIC,
            DesiredMotorOrientation,
            callback=self.__desired_orientation_callback,
            queue_size=constants.CONTROLS_PKG_SUBSCRIBER_QUEUE_SIZE
        )

        self.current_orientation_publisher = rospy.Publisher(
            constants.CONTROLS_PKG_CURRENT_ORIENTATION_TOPIC,
            CurrentMotorOrientation,
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

        This method puts the program in an infinite
        loop, during each iteration of which the node publishes
        the current orientation of the Moteus motors
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

            msg = CurrentMotorOrientation(numMotors=self.num_motors, states=motor_states)
            self.current_orientation_publisher.publish(msg)
            rate.sleep()

    def shutdown_moteus(self):
        """Safely terminates the node
        and performs all cleanup necessary
        with the Moteus motors and ROS

        It is important to invoke this method
        before terminating the program; otherwise the
        Moteus motors in communication with this class
        will not be properly stopped and there is a
        possible chance of hardware damage
        """
        self.moteus_instance.closeMoteus()
        exit()

    def __desired_orientation_callback(self, msg: DesiredMotorOrientation):
        """A method which serves as the callback function
        of the ROS subscriber representing this instance
        as a listener to the 'desired_orientation' topic

        @param msg: A message emitted by the Intel NUC
        of type 'MotorOrientation' describing how the motors
        should be re-oriented
        """
        if len(self.id_map) != msg.numMotors:
            raise ValueError('Expected a exactly one desired orientation for each motor')

        # The state of the motor at index 'msg_index' corresponds to the
        # (msg_index + 1)st motor in the dictionary mapping motor ids to
        # their CAN bus lanes
        for msg_index, motor_id in enumerate(self.id_map.keys()):
            motor_state = msg.states[msg_index]
            self.moteus_instance.setAttributes(motor_id, motor_state.position, motor_state.velocity, motor_state.torque)


    # MARK - Python Magic Methods -

    def __str__(self):
        """Outputs a formatted string describing the instance"""
        name = 'RaspberryPiMoteusWrapper\n'
        subscribed_to = 'subscribed to: ' + constants.CONTROLS_PKG_DESIRED_ORIENTATION_TOPIC + "\n"
        pub_to = 'publishing to: ' + constants.CONTROLS_PKG_CURRENT_ORIENTATION_TOPIC
        return name + subscribed_to + pub_to

# Connect to the ROS subsystem
rospy.init_node(constants.CONTROLS_PKG_RASPBERRYPI_NODE_NAME)
wrapper = RaspberryPiMoteusWrapper(motor_list=constants.CONTROLS_PKG_BUS_CONFIGURATION)

# Add a signal handler for SIGINT (^C)
signal.signal(signal.SIGINT, lambda sig, frame: wrapper.shutdown_moteus())

# Run the node. The try-except ensures that if anything unexpected error is
# raised that the Moteus motors are properly notified to stop
try:
    wrapper.run_motor_orientation_loop()
except:
    wrapper.shutdown_moteus()