import sys
import os
proto_dir = os.path.join(os.path.dirname(__file__), "proto")
if proto_dir not in sys.path:
    sys.path.insert(0, proto_dir)

from proto.ProtoMain_pb2 import MainMessage

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

