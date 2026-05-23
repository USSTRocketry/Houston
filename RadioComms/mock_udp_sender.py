import socket
import struct
import time
import math
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'DisplaySystem'))

from proto.ProtoMain_pb2 import MainMessage

MULTICAST_GROUP = ('239.255.0.1', 33333)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.2)
ttl = struct.pack('b', 1)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

print(f"Sending Mock Protobuf UDP Multicast to {MULTICAST_GROUP[0]}:{MULTICAST_GROUP[1]}")

timestamp = 0
time_sec = 0.0

while timestamp < 10000: # run for 10 seconds worth
    msg = MainMessage()
    fd = msg.in_flight_data
    fd.timestamp_ms = timestamp
    
    # Simulate some movement
    fd.bmp_data.altitude = 100.0 + 50.0 * math.sin(time_sec * 0.5)
    fd.bmp_data.temperature = 25.0 + 2.0 * math.cos(time_sec * 0.1)
    fd.bmp_data.pressure = 1013.25 - 0.12 * fd.bmp_data.altitude
    
    fd.accel.X = math.sin(time_sec)
    fd.accel.Y = math.cos(time_sec)
    fd.accel.Z = -9.8 + math.sin(time_sec * 0.2)
    
    fd.gyro.X = 0.1 * math.sin(time_sec)
    fd.gyro.Y = 0.1 * math.cos(time_sec)
    fd.gyro.Z = 0.05 * math.sin(time_sec * 0.5)
    
    fd.magnetometer.X = 0.5
    fd.magnetometer.Y = 0.2
    fd.magnetometer.Z = 0.8
    
    fd.accel_gyro_temperature = 30.0
    fd.thermometer = 20.0
    
    data = msg.SerializeToString()
    sock.sendto(data, MULTICAST_GROUP)
    
    print(f"Sent {len(data)} bytes, Alt: {fd.bmp_data.altitude:.2f}m")
    
    time.sleep(0.5)
    timestamp += 500
    time_sec += 0.5

print("Finished sending.")
