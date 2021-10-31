class MoteusMotorDescriptor:
    """Describes an immutable configuration for a single
    Moteus motor

    The Moteus motors communicate with the RaspberryPi via the
    CAN protocol. Among the information needed to successfully
    establish a communication between the RaspberryPi hardware and
    the Moteus motors is a description of how and where each motor is
    communicating, as well as which motor is which. This class
    captures the relevant information needed to sufficiently describe
    the connection between a Moteus motor and the RaspberryPi under
    the CAN communication protocol
    """

    def __init__(self, name, hardware_id, can_bus_lane_id):
        """Creates a new descriptor for a motor with then given
        human-readable name _name_ and CAN communication information

        @param name: The human-readable name assigned to this
        particular motor. This serves as a convenience when talking
        about particular motors. Instead of talking about 'motor with id
        1', we can instead talk about 'the left motor'

        @param can_bus_lane_id: The ID specifying which CAN bus this
        motor is attached to for communication purposes

        @param: motor_id: A fixed hardware identifier assigned to each
        motor that corresponds to the motor that is attached to CAN bus
        _can_bus_lane_id_ and should be identified with the name _name_
        according to this descriptor
        """
        self.name = name
        self.can_bus_lane_id = can_bus_lane_id
        self.hardware_id = hardware_id