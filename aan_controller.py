from starling import NexusSubscriber, NexusPublisher
from scipy.interpolate import PchipInterpolator
from states import IMUState, ActuatorState, ControllerState, GaitState
import time
import msgspec

STANCE = 0
SWING = 1
IDLE = 2

OPERATION_FREQUENCY = 200

TORQUE_LIMIT = (-10.0, 10.0) # Nm

# State Variables
shank_state = IMUState()
thigh_state = IMUState()
controller_state = ControllerState()
gait_state = GaitState()
motor_state = ActuatorState()
target_angle = PchipInterpolator([0, 15, 20, 65, 73, 90, 100], [0, 0, 0, 0, 0, 0, 0])

sub = NexusSubscriber()

decoder = msgspec.json.Decoder()
encoder = msgspec.json.Encoder()

sub.subscribe("imus.thigh.state", lambda msg, topic: thigh_state.update(decoder.decode(msg)))
sub.subscribe("imus.shank.state", lambda msg, topic: shank_state.update(decoder.decode(msg)))
sub.subscribe("actuator.state", lambda msg, topic: motor_state.update(decoder.decode(msg)))
sub.subscribe("calibration.knee_angle", lambda msg, topic: controller_state.update(decoder.decode(msg)))
sub.subscribe("controller.settings", lambda msg, topic: controller_state.update(decoder.decode(msg)))
sub.subscribe("gait.state", lambda msg, topic: gait_state.update(decoder.decode(msg)))

pub = NexusPublisher()

knee_angle = 0.0
error = 0.0

while True:
    # itter_time = time.clock_gettime(clk_id=time.CLOCK_MONOTONIC)
    itter_time = time.perf_counter()
    target_angle_value = target_angle(gait_state.gait_phase).item()
    knee_angle = (shank_state.roll - thigh_state.roll) - controller_state.calibration_offset
    error = target_angle_value - knee_angle

    command_torque = 0.0
    if (gait_state.gait_state == SWING):
        if (abs(error) > controller_state.tolerance_swing/2):
            command_torque = (error * controller_state.kp_swing)
    elif (gait_state.gait_state == STANCE):
        if (abs(error) > controller_state.tolerance_stance/2):
            command_torque = (error * controller_state.kp_stance)
    elif (gait_state.gait_state == IDLE):
        command_torque = 0.0

    command_torque = max(TORQUE_LIMIT[0], min(TORQUE_LIMIT[1], command_torque))
    controller_state.update({"knee_angle": knee_angle, "target_angle": target_angle_value, "error": error, "trq_cmd": command_torque})

    pub.send("actuator.command", encoder.encode({"trq_cmd": command_torque}))

    # Publish a snapshot to be logged
    pub.send("snapshot", encoder.encode({
        "time": itter_time,
        "shank_state": shank_state.__dict__,
        "thigh_state": thigh_state.__dict__,
        "controller_state": controller_state.__dict__,
        "actuator_state": motor_state.__dict__,
        "gait_state": gait_state.__dict__
    }))
    time.sleep(1/230)