from epicallypowerful.sensing import MicrostrainImus
from epicallypowerful.toolbox import TimedLoop
from states import IMUState
from starling import NexusPublisher, msgspec
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


imu_ids = ["111111", "222222"] # Example IMU IDs
imu_names = None
frequency = 100

if imu_names is None:
    imu_names = imu_ids

pub = NexusPublisher()
imus = MicrostrainIMUs(imu_ids)
imu_states = [IMUState() for _ in imu_ids]
loop = TimedLoop(frequency)
encoder = msgspec.json.Encoder()
error_state = False


while loop():
    try:
        imu_data = [imus.get_data(imu_id) for imu_id in imu_ids]
        error_state = False
    except Exception as e:
        if not error_state:
            logger.error(f"Error retrieving IMU data: {e}, check connections and cable integrity.")
            error_state = True
        continue
    for i, data in enumerate(imu_data):
        imu_states[i].update({
            "accx": data.accx,
            "accy": data.accy,
            "accz": data.accz,
            "gyrox": data.gyrox,
            "gyroy": data.gyroy,
            "gyroz": data.gyroz,
            "magx": data.magx,
            "magy": data.magy,
            "magz": data.magz,
            "orient_w": data.orient_w,
            "orient_x": data.orient_x,
            "orient_y": data.orient_y,
            "orient_z": data.orient_z,
            "roll": data.roll,
            "pitch": data.pitch,
            "yaw": data.yaw
        })
        pub.send(f"imus.{imu_names[i]}.state", encoder.encode(imu_states[i].__dict__))
    