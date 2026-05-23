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
            "timestamp": fd.timestamp_ms / 1000.0 if fd.timestamp_ms else 0, # Convert ms to s
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
            "flight_time": fd.timestamp_ms / 1000.0 if fd.timestamp_ms else 0
        }
    
    # Return empty dict if not in_flight_data or log message
    return {}
