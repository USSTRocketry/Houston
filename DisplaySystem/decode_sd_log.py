#!/usr/bin/env python3
import sys
import os
import warnings

# Suppress protobuf warnings
warnings.filterwarnings("ignore")

# Ensure display_main directory and proto directory are in sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
proto_dir = os.path.join(current_dir, "proto")
if proto_dir not in sys.path:
    sys.path.insert(0, proto_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

try:
    import ProtoMain_pb2 as pb_main
    import RocketGroundCommunication_pb2 as pb_rgc
except ImportError:
    from proto import ProtoMain_pb2 as pb_main
    from proto import RocketGroundCommunication_pb2 as pb_rgc

STATE_NAMES = {
    0: "UNKNOWN",
    1: "UNARMED",
    2: "GROUNDIDLE",
    3: "INFLIGHT",
    4: "MAINCHUTE",
}

def decode_varint(data, offset):
    res = 0
    shift = 0
    start = offset
    while offset < len(data):
        b = data[offset]
        offset += 1
        res |= (b & 0x7f) << shift
        if not (b & 0x80):
            break
        shift += 7
        if shift >= 64:
            return None, start
    return res, offset

def parse_sd_bin(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' not found.")
        return

    with open(file_path, "rb") as f:
        data = f.read()

    print(f"\n=== Reading SD Log File: {os.path.basename(file_path)} ({len(data)} bytes) ===")
    print(f"{'Idx':<4} | {'Time (s)':<8} | {'State':<10} | {'BMP Alt(m)':<10} | {'Press(hPa)':<10} | {'Temp(C)':<8} | {'Accel (X,Y,Z)':<22} | {'GPS (Lat, Lon, Sats)':<22}")
    print("-" * 115)

    offset = 0
    record_count = 0

    while offset < len(data):
        payload_len, next_offset = decode_varint(data, offset)
        
        # Check if varint length is valid and payload fits in data
        if payload_len is None or payload_len <= 0 or payload_len > 1024 or (next_offset + payload_len) > len(data):
            offset += 1
            continue

        payload = data[next_offset : next_offset + payload_len]
        log_msg = pb_main.LogMessage()

        try:
            log_msg.ParseFromString(payload)
            if log_msg.HasField("main_message"):
                mm = log_msg.main_message
                if mm.HasField("in_flight_data"):
                    fd = mm.in_flight_data
                    ts_sec = mm.timestamp / 1000.0 if mm.timestamp else 0.0
                    
                    state_val = fd.flight_state if hasattr(fd, "flight_state") else 0
                    state_str = STATE_NAMES.get(state_val, "UNKNOWN")

                    bmp_alt = fd.bmp_data.altitude if fd.HasField("bmp_data") else 0.0
                    bmp_press = fd.bmp_data.pressure if fd.HasField("bmp_data") else 0.0
                    bmp_temp = fd.bmp_data.temperature if fd.HasField("bmp_data") else 0.0
                    
                    ax = fd.accel.X if fd.HasField("accel") else 0.0
                    ay = fd.accel.Y if fd.HasField("accel") else 0.0
                    az = fd.accel.Z if fd.HasField("accel") else 0.0

                    gps_str = "No Fix"
                    if hasattr(fd, "gps_data") and fd.HasField("gps_data"):
                        gps = fd.gps_data
                        if getattr(gps, "fix_quality", 0) > 0:
                            gps_str = f"{gps.latitude:.4f},{gps.longitude:.4f} ({gps.satellites}s)"
                        else:
                            sats = getattr(gps, "satellites", 0)
                            gps_str = f"No Fix ({sats}s)" if sats > 0 else "No Fix"

                    record_count += 1
                    print(f"{record_count:<4} | {ts_sec:<8.2f} | {state_str:<10} | {bmp_alt:<10.2f} | {bmp_press:<10.2f} | {bmp_temp:<8.2f} | ({ax:.1f},{ay:.1f},{az:.1f})".ljust(88) + f" | {gps_str}")
                    offset = next_offset + payload_len
                    continue
        except Exception:
            pass

        offset += 1

    print("-" * 115)
    print(f"Total Telemetry Records Extracted: {record_count}\n")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python decode_sd_log.py <path_to_FlightData.bin>")
    else:
        parse_sd_bin(sys.argv[1])
