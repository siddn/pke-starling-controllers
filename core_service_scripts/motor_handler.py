# from epicallypowerful.actuation import CyberGear, ActuatorGroup
# from epicallypowerful.toolbox import TimedLoop
import time
from starling import NexusSubscriber, NexusPublisher, msgspec
from states import ActuatorState, RMSTrack
import numpy as np
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

OPERATION_FREQUENCY = 200

LEFT = 0x1
RIGHT = 0x2

PI = np.pi

HARDSTOPS = (-20*PI/180, 180*PI/180) # rads
TORQUE_ERROR_LIMIT = 100.0 # Nm*s (?)

# State variables
motor_state = ActuatorState(id=RIGHT) # Default to right side assumption
connection_expected = False
motors = ActuatorGroup([CyberGear(LEFT), CyberGear(RIGHT)])
torque_tracking_error: RMSTrack = RMSTrack(int(0.1*OPERATION_FREQUENCY)) # 100ms window

encoder = msgspec.json.Encoder()
decoder = msgspec.json.Decoder()

sub = NexusSubscriber()
sub.subscribe("actuator.command", lambda msg, topic: motor_state.update(decoder.decode(msg)))

pub = NexusPublisher()
while True:
    itter_time = time.clock_gettime(clk_id=time.CLOCK_MONOTONIC)

    left_connected, right_connected = motors.is_connected(LEFT), motors.is_connected(RIGHT)

    if (left_connected and right_connected): # Both motors should not be connected at the same time
        motor_state.actuation_enabled = False
        connection_expected = True
        logger.error("BOTH MOTORS ARE CONNECTED: DISCONNECT ONE TO RESUME OPERATION")
    elif (left_connected or right_connected):
        connection_expected = True
        motor_state.id = LEFT if left_connected else RIGHT
        motor_data = motors.get_data(motor_state.id)
        motor_state.update({
            "pos_state": motor_data.current_position,
            "vel_state": motor_data.current_velocity,
            "trq_state": motor_data.current_torque,
            "time": itter_time
        })
        torque_tracking_error.add_point(
            ((motor_state.trq_cmd - motor_state.trq_state) / motor_state.trq_cmd) if motor_state.trq_cmd != 0 else 0.0
        )

    ### Safety checks ###
    if ((not left_connected) and (not right_connected)) and connection_expected:
        # Logic to trigger the MOSFET power cycling. This will open and close the power circuit briefly, killing any static torque command that can't be disabled via the CAN communication.
        motor_state.actuation_enabled = False
        connection_expected = False
        logger.warning("Motors not responding, power cycling and disabling actuation...")
    if (motor_state.pos_state < HARDSTOPS[0] or motor_state.pos_state > HARDSTOPS[1]):
        motor_state.actuation_enabled = False
        logger.warning("Virtual hardstops exceeded, disabling actuation...")
    if (torque_tracking_error.current_rms >= TORQUE_ERROR_LIMIT):
        logger.warning("Torque tracking error limit exceeded, disabling actuation...")
        motor_state.actuation_enabled = False
    if not motor_state.actuation_enabled:
        motor_state.update({ "pos_cmd": 0.0, "vel_cmd": 0.0, "trq_cmd": 0.0, "kp": 0.0, "kd": 0.0 })

    ### Command the motor ###
    motors.set_torque(motor_state.id, motor_state.trq_cmd)
    pub.send("actuator.state", encoder.encode(motor_state.__dict__))

    time.sleep(1)