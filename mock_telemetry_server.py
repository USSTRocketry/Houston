import socket
import json
import time
import random
import threading
import math
import numpy as np

# Configuration
HOST = '0.0.0.0'
PORT = 33333
DT = 0.5 # Simulation Step (Seconds) - Sync with Broadcast Interval

class RocketPhysics:
    def __init__(self):
        # Initial State (Local NED frame usually, but we'll stick to simple Cartesian)
        # Position (meters) relative to Launchpad
        self.pos = np.array([0.0, 0.0, 0.0]) # x(East), y(North), z(Up)
        self.vel = np.array([0.0, 0.0, 0.0])
        
        # Orientation (Euler Angles in Degrees for simplicity)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Constants
        self.MASS = 50.0  # kg
        self.GRAVITY = np.array([0.0, 0.0, -9.81])
        
        # Base Lat/Lon (Clear Lake)
        self.BASE_LAT = 29.5646
        self.BASE_LON = -95.0945
        
        # Flight Phase
        self.state = 0 # 0:Idle, 1:Boost, 2:Coast/Apogee, 3:Drogue, 4:Main (Landed)
        self.flight_start_time = 0
        self.flight_end_time = 0
        self.launch_initiated = False

    def get_rotation_matrix(self):
        # Convert Euler to Rotation Matrix (Body to World)
        # Simplified for small angles or standard aerospace sequence (ZYX)
        r = math.radians(self.roll)
        p = math.radians(self.pitch)
        y = math.radians(self.yaw)
        
        # Rx (Roll)
        Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
        # Ry (Pitch)
        Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
        # Rz (Yaw)
        Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
        
        # R = Rz * Ry * Rx
        return Rz @ Ry @ Rx

    def update(self):
        current_time = time.time()
        
        thrust_body = np.array([0.0, 0.0, 0.0])
        drag_coeff = 0.02
        
        if self.state == 0: # Idle
            self.pos = np.array([0.0, 0.0, 5.0])
            self.vel = np.array([0.0, 0.0, 0.0])
            # Random jitter in orientation
            self.pitch = random.uniform(-1, 1)
            self.yaw = random.uniform(0, 360)
            
            if self.launch_initiated:
                 self.state = 1
                 self.flight_start_time = current_time
                 print("LAUNCH!")
            elif random.random() < 0.01:
                self.launch_initiated = True # Auto launch for testing
                 
        elif self.state == 1: # Boost
            thrust_img = 5500.0 # Tuned for ~16k ft / ~1200 ft/s
            thrust_body = np.array([0.0, 0.0, thrust_img])
            
            # Tilt to generate horizontal velocity (~200 ft/s)
            # Need approx 8 deg tilt
            if (current_time - self.flight_start_time) < 0.5: # Initial Kick
                 self.pitch = 8.0 + random.uniform(-1, 1)
            
            self.pitch += random.uniform(-0.1, 0.1) 
            self.roll += random.uniform(-1, 1)
            
            if (current_time - self.flight_start_time) > 5.0: # 5s Burn
                self.state = 2
                print("MECO / COAST START")

        elif self.state == 2: # Coast / Apogee
            thrust_body = np.array([0.0, 0.0, 0.0])
            # Drag is higher
            
            if self.vel[2] < -5.0: # Falling significantly
                self.state = 3
                print("DROGUE DEPLOY")

        if self.state == 3: # Drogue/Descent
            # Simulate high drag (terminal velocity ~20 m/s)
            drag_coeff = 2.0 
            
            if self.pos[2] < 200: # Main chute altitude
                self.state = 3 # Keep 3 for now, or add 3.5
                drag_coeff = 5.0 # Main Chute
                
            if self.pos[2] <= 5.0:
                self.state = 4
                self.pos[2] = 5.0
                self.flight_end_time = current_time # Capture landing time
                print("TOUCHDOWN")
        
        elif self.state == 4: # Landed
             self.vel = np.array([0.0, 0.0, 0.0])
             thrust_body = np.array([0.0, 0.0, 0.0])

        # 2. Physics Integration (Euler)
        if self.state != 0 and self.state != 4:
            R = self.get_rotation_matrix()
            
            # Forces in World Frame
            # Thrust
            F_thrust = R @ thrust_body
            
            # Gravity
            F_gravity = self.GRAVITY * self.MASS
            
            # Drag (opposes velocity)
            # F_drag = -0.5 * rho * v^2 * Cd * A
            # Simplified: F_drag = -k * vel * |vel|
            vel_mag = np.linalg.norm(self.vel)
            if vel_mag > 0:
                F_drag = -drag_coeff * self.vel * vel_mag
            else:
                F_drag = np.array([0.0, 0.0, 0.0])
            
            # Total Force
            F_total = F_thrust + F_gravity + F_drag
            
            # Acceleration (World)
            accel_world = F_total / self.MASS
            
            # Integrate
            self.vel += accel_world * DT
            self.pos += self.vel * DT
            
            # Ground Collision
            if self.pos[2] < 0:
                self.pos[2] = 0
                self.vel = np.array([0.0, 0.0, 0.0])
                if self.state > 1:
                    self.state = 4
            
            # 3. Simulate IMU (Accelerometer measures Specific Force, not net accel!)
            # Accel_measured = R_world_to_body * (Accel_world - Gravity)
            # Note: Gravity vector is down (-9.81). So Accel_world - Gravity 
            # If stationary: 0 - (-9.81) = +9.81 up (Reaction force)
            
            specific_force_world = accel_world - self.GRAVITY
            
            # Rotate into Body Frame for sensor output
            # R is Body->World. Transpose is World->Body
            accel_sensor = R.T @ specific_force_world
            
            # Add Noise
            accel_sensor += np.random.normal(0, 0.2, 3)
            
        else: # Stationary
            # On Pad or Landed
            # Accel measures reaction force (Gravity rotated)
            R = self.get_rotation_matrix()
            reaction_world = -self.GRAVITY # +9.81 Up
            accel_sensor = R.T @ reaction_world
            accel_sensor += np.random.normal(0, 0.05, 3)

        # 4. Generate Telemetry Dict
        
        # Lat/Lon Conversion (Approximate meters to degrees)
        # 1 deg lat ~ 111km
        lat_deg = self.BASE_LAT + (self.pos[1] / 111132.0)
        lon_deg = self.BASE_LON + (self.pos[0] / (111412.0 * math.cos(math.radians(lat_deg))))
        
        flight_time = 0.0
        if self.state > 0:
            if self.state == 4 and self.flight_end_time > 0:
                flight_time = self.flight_end_time - self.flight_start_time
            else:
                flight_time = current_time - self.flight_start_time
            
        # Gyro (Angular Velocity) - Simplified/Fake
        # Just noise unless we implement rotational dynamics (torques)
        gyro_data = [random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1)]
        
        return {
            "bmp_temp": 25.0 + random.uniform(-0.5, 0.5),
            "pressure": 1013.25 * math.exp(-0.00012 * self.pos[2]),
            "bmp_alt": self.pos[2],
            "accel_x": accel_sensor[0],
            "accel_y": accel_sensor[1],
            "accel_z": accel_sensor[2],
            "gyro_x": gyro_data[0],
            "gyro_y": gyro_data[1],
            "gyro_z": gyro_data[2],
            "imu_temp": 30.0,
            "mag_x": 0, "mag_y": 0, "mag_z": 0,
            "extra_temp": 20.0,
            "lat": lat_deg,
            "lon": lon_deg,
            "gps_alt": self.pos[2],
            "gps_speed": np.linalg.norm(self.vel),
            "gps_angle": 0.0,
            "timestamp": int(current_time),
            "state": self.state,
            "flight_time": flight_time
        }

def start_server():
    sim = RocketPhysics()
    
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"Physics Server Listening on {HOST}:{PORT}...")
    
    while True:
        try:
            conn, addr = server.accept()
            print(f"Connected by {addr}")
            
            while True:
                data = sim.update()
                json_str = json.dumps(data) + "\n"
                
                try:
                    conn.sendall(json_str.encode('utf-8'))
                    time.sleep(DT)
                except BrokenPipeError:
                    print("Client disconnected.")
                    break
                except Exception as e:
                    print(f"Error: {e}")
                    break
            
            conn.close()
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Server Error: {e}")
            time.sleep(1)
    
    server.close()

if __name__ == "__main__":
    start_server()
