from starling import NexusSubscriber, NexusPublisher
from scipy.interpolate import PchipInterpolator
from core_service_scripts.states import IMUState, ActuatorState, ControllerState, GaitState
import time
import msgspec
from epicallypowerful.toolbox import TimedLoop

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

def update_settings(msg, topic):
    msg = decoder.decode(msg)
    controller_state.update(msg)
    global target_angle
    if "stance_angle" in msg or "swing_angle" in msg:
        st_angle = controller_state.stance_angle
        sw_angle = controller_state.swing_angle
        target_angle = PchipInterpolator(
            [0, 15, 20, 65, 73, 90, 100],
            [st_angle, st_angle, st_angle, st_angle, sw_angle, sw_angle, sw_angle]
        )

def calibrate_knee_angle(msg, topic):
    msg = decoder.decode(msg)
    baseline_knee_angle = controller_state.knee_angle # Use the current knee angle as the reference
    calibration_offset = msg.get("calibration_offset", 0.0)
    controller_state.update({"calibration_offset": baseline_knee_angle - calibration_offset})

sub.subscribe("imus.thigh.state", lambda msg, topic: thigh_state.update(decoder.decode(msg)))
sub.subscribe("imus.shank.state", lambda msg, topic: shank_state.update(decoder.decode(msg)))
sub.subscribe("actuator.state", lambda msg, topic: motor_state.update(decoder.decode(msg)))
sub.subscribe("calibration.knee_angle", calibrate_knee_angle)
sub.subscribe("controller.settings", update_settings)
sub.subscribe("gait.state", lambda msg, topic: gait_state.update(decoder.decode(msg)))

pub = NexusPublisher()

knee_angle = 0.0
error = 0.0

loop = TimedLoop(frequency=OPERATION_FREQUENCY)

while loop.continue_loop():
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
