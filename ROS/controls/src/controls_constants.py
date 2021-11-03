
# The maximum number of ROS messages that
# are buffered by publishers in the controls package
CONTROLS_PKG_PUBLISHER_QUEUE_SIZE = 10

# The maximum number of ROS messages that
# are buffered by subscribers in the controls package
CONTROLS_PKG_SUBSCRIBER_QUEUE_SIZE = 10

# The topic name through which the desired orientation
# of the motors is communicated
CONTROLS_PKG_DESIRED_ORIENTATION_TOPIC = 'desired_orientation'

# The topic name through which the current orientation
# of the motors is communicated
CONTROLS_PKG_CURRENT_ORIENTATION_TOPIC = 'current_orientation'

# The default rate at which the Moteus motors are queried
# for their current state (defaults to 60 times a second)
CONTROLS_PKG_DEFAULT_HW_REFRESH_RATE = 60

# A mapping of the Moteus motor IDs with CAN bus lanes
CONTROLS_PKG_BUS_CONFIGURATION = {1 : 0, 2 : 0}

# The name of the node that runs on the RaspberryPi
CONTROLS_PKG_RASPBERRYPI_NODE_NAME = 'controls_raspberrypi_node'