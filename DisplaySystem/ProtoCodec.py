import sys
import os
import struct

current_dir = os.path.dirname(os.path.abspath(__file__))
proto_dir = os.path.join(current_dir, "proto")
if proto_dir not in sys.path:
    sys.path.insert(0, proto_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

try:
    import ProtoMain_pb2
    from ProtoMain_pb2 import MainMessage
except ImportError:
    from proto.ProtoMain_pb2 import MainMessage

# MainMessage.switch_radio_frequency field number; Ack.response_to_which_message uses this tag.
SWITCH_RADIO_FREQUENCY_TAG = 3
# WiFi-only command for Radisson: magic + little-endian float32 MHz. Not forwarded over LoRa.
INTERLINK_FREQ_MAGIC = b"ILFQ"

def ProtoDecode(buffer: bytes):
    msg = MainMessage()
    try:
        msg.ParseFromString(buffer)
    except Exception as e:
        print(f"Protobuf decode error: {e}")
        return None

    if msg.HasField("in_flight_data"):
        fd = msg.in_flight_data
        return {
            "timestamp": msg.timestamp / 1000.0 if msg.timestamp else 0, # Convert ms to s
            "bmp_temp": fd.bmp_data.temperature,
            "pressure": fd.bmp_data.pressure,
            "bmp_alt": fd.bmp_data.altitude,
            "accel_x": fd.accel.X,
            "accel_y": fd.accel.Y,
            "accel_z": fd.accel.Z,
            "gyro_x": fd.gyro.X,
            "gyro_y": fd.gyro.Y,
            "gyro_z": fd.gyro.Z,
            "imu_temp": fd.accel_gyro_temperature,
            "mag_x": fd.magnetometer.X,
            "mag_y": fd.magnetometer.Y,
            "mag_z": fd.magnetometer.Z,
            "extra_temp": fd.thermometer,
            # Defaults for missing data like GPS and State (since they are not in InFlightData)
            "lat": 29.5646,
            "lon": -95.0945,
            "gps_alt": fd.bmp_data.altitude,
            "gps_speed": 0,
            "gps_angle": 0,
            "state": 1, # Just simulate boost state to see colors change
            "flight_time": msg.timestamp / 1000.0 if msg.timestamp else 0
        }
    
    if msg.HasField("command_msg"):
        return {
            "command": msg.command_msg.command
        }

    if msg.HasField("ack"):
        return {
            "ack": msg.ack.response_to_which_message
        }

    if msg.HasField("switch_radio_frequency"):
        return {
            "new_frequency": msg.switch_radio_frequency.new_frequency
        }

    # Return empty dict if not in_flight_data or log message
    return {}


def encode_command(command_id: int, timestamp_ms: int = 0) -> bytes:
    """
    Encode a command message into protobuf binary format.
    MainMessage:
      - timestamp (field 1, varint): timestamp_ms
      - command_msg (field 8, length-delimited submessage):
        - command (field 1, varint): command_id
    """
    def to_varint(val: int) -> bytes:
        res = bytearray()
        while val >= 0x80:
            res.append((val & 0x7F) | 0x80)
            val >>= 7
        res.append(val & 0x7F)
        return bytes(res)

    submsg = bytes([0x08]) + to_varint(command_id)
    ts_field = bytes([0x08]) + to_varint(timestamp_ms)
    cmd_field = bytes([0x42, len(submsg)]) + submsg

    return ts_field + cmd_field


def encode_switch_frequency(frequency_mhz: float, timestamp_ms: int = 0) -> bytes:
    msg = MainMessage()
    msg.timestamp = timestamp_ms
    msg.switch_radio_frequency.new_frequency = frequency_mhz
    return msg.SerializeToString()


def encode_ack(response_to_which_message: int, timestamp_ms: int = 0) -> bytes:
    msg = MainMessage()
    msg.timestamp = timestamp_ms
    msg.ack.response_to_which_message = response_to_which_message
    return msg.SerializeToString()


def encode_interlink_frequency(frequency_mhz: float) -> bytes:
    return INTERLINK_FREQ_MAGIC + struct.pack("<f", frequency_mhz)

