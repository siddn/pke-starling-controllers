# from epicallypowerful.actuation import CyberGear, ActuatorGroup
# from epicallypowerful.toolbox import TimedLoop
import time
import msgspec
from starling import NexusSubscriber, NexusPublisher
from states import ActuatorState, RMSTrack

OPERATION_FREQUENCY = 200

LEFT = 1
RIGHT = 2

HARDSTOPS = (-20, 180) # deg
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

    if (motors.is_connected(LEFT) and motors.is_connected(RIGHT)):
        motor_state.actuation_enabled = False
        connection_expected = True
        print("BOTH MOTORS ARE CONNECTED: DISCONNECT ONE TO RESUME OPERATION")

    if (motors.is_connected(LEFT) or motors.is_connected(RIGHT)):
        connection_expected = True
        motor_state.id = LEFT if motors.is_connected(LEFT) else RIGHT
        motor_data = motors.get_data(motor_state.id)
        motor_state.update({
            "pos_state": motor_data.current_position,
            "vel_state": motor_data.current_velocity,
            "trq_state": motor_data.current_torque,
            "time": itter_time
        })
        torque_tracking_error.add_point(motor_state.trq_cmd - motor_state.trq_state)
    # Safety checks
    if ((not motors.is_connected(LEFT)) and (not motors.is_connected(RIGHT))) and connection_expected:
        # Logic to trigger the MOSFET power cycling. This will open and close the power circuit briefly, killing any static torque command that can't be disabled via the CAN communication.
        motor_state.actuation_enabled = False
        connection_expected = False
    if (motor_state.pos_state < HARDSTOPS[0] or motor_state.pos_state > HARDSTOPS[1]):
        motor_state.actuation_enabled = False
    if (motor_state.torque_tracking_error.current_rms >= TORQUE_ERROR_LIMIT):
        motor_state.actuation_enabled = False

    if not motor_state.actuation_enabled:
        motor_state.update({ "pos_cmd": 0.0, "vel_cmd": 0.0, "trq_cmd": 0.0, "kp": 0.0, "kd": 0.0 })

    motors.set_torque(motor_state.id, motor_state.trq_cmd)

    pub.send("actuator.state", encoder.encode(motor_state.__dict__))

    time.sleep(1)