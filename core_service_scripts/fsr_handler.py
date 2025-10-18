from starling import NexusPublisher, msgspec
from epicallypowerful.toolbox import TimedLoop
from collections import deque
import logging
import RPi.GPIO as GPIO
from states import GaitState
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

NAN = np.nan
FSR_PIN = 17
FREQUENCY = 100
MAX_DUR = 3.0; MIN_DUR = 0.5
STANCE = 1; SWING = 0; IDLE = -1


GPIO.setmode(GPIO.BOARD)
GPIO.setup(FSR_PIN, GPIO.IN)

pub = NexusPublisher()
loop = TimedLoop(FREQUENCY)
encoder = msgspec.json.Encoder()
error_state = False

gait_durations = deque([1.5]*3, maxlen=3) # Initialize with default gait durations
fsr_samples = deque([0]*3, maxlen=3)
gait_state = GaitState()
t_hc = time.perf_counter()
t_last_event = 0.0

while loop():
    try:
        fsr_value = GPIO.input(FSR_PIN)
        error_state = False
    except Exception as e:
        if not error_state:
            logger.error(f"Error retrieving FSR data: {e}, check connections and cable integrity.")
        continue

    # Check the fsr value, and update any state transitions
    t_now = time.perf_counter()
    stride_duration = t_now - t_hc
    # Check for state transitions
    if stride_duration < MIN_DUR or stride_duration > MAX_DUR: 
        # This likey indicates standing or a stutter step.
        # Either way, we need to treat this as bad data, and force three consecutive good steps to re-engage walking
        fsr_samples.append(NAN)
    else:
        fsr_samples.append(fsr_value)
    # ANY -> IDLE
    if sum(fsr_samples) == NAN:
        gait_state.gait_state = IDLE
    # IDLE -> STANCE or SWING -> STANCE
    elif gait_state.gait_state != STANCE and sum(fsr_samples) == 3:
        gait_state.gait_state = STANCE
        # Update gait duration tracking
        # ADD TIMING STUFF LATER
    # STANCE -> SWING
    elif gait_state.gait_state == STANCE and sum(fsr_samples) == 0:
        gait_state.gait_state = SWING

    pub.send(f"gait.state", encoder.encode({"value": fsr_value}))
