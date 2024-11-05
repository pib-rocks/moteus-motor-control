import asyncio
import time
from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_imu_v3 import BrickletIMUV3
from moteus import Controller
import math
import numpy as np
import control as ct

HOST = "localhost"
PORT = 4223
UID = "2bS8"  # Replace UID

async def main():
    ipcon = IPConnection()
    imu = BrickletIMUV3(UID, ipcon)
    ipcon.connect(HOST, PORT)

    # Initialize controllers
    controller_left = Controller(id=1)
    controller_right = Controller(id=2)

    # System parameters for inverted pendulum
    m = 0.4   # mass of the robot in kg
    l = 0.2   # length of the tobot in meters
    g = 9.81  # gravity acceleration in m/s^2
    I = m * l**2  # moment of inertia

    # State-space representation
    A = np.array([[0, 1],
                  [g / l, 0]])
    B = np.array([[0],
                  [1 / I]])

    # LQR weighting matrices
    Q = np.array([[100, 0],
                  [0, 1]])
    R = np.array([[1]])

    # Compute LQR gain
    K, _, _ = ct.lqr(A, B, Q, R)

    # Scale factor to map control input to velocity command
    scale_factor = 0.05  # Adjust as necessary

    # Deadband thresholds
    deadband_angle_deg = 0.4  # degrees
    print("Starting balancing robot with LQR control. Press Ctrl+C to exit.")

    try:
        while True:
            # Read IMU orientation (pitch angle) and angular velocity
            try:
                _, _, pitch = imu.get_orientation()
                pitch_angle_deg = (pitch / 100.0)*6.3  # Pitch angle in degrees
                pitch_angle_rad = math.radians(pitch_angle_deg)  # Convert to radians

                angular_velocity_x, _, _ = imu.get_angular_velocity()
                # Angular velocity around x-axis (pitch rate), in degrees per second
                pitch_rate_deg_per_sec = angular_velocity_x / 16.0
                pitch_rate_rad_per_sec = math.radians(pitch_rate_deg_per_sec)
            except Exception as e:
                print(f"Error reading IMU data: {e}")
                continue

            # Apply deadband
            if abs(pitch_angle_deg) < deadband_angle_deg:
                command = 0.0
            else:
                # Form state vector x
                x = np.array([[pitch_angle_rad],
                              [pitch_rate_rad_per_sec]])

                # Compute control input u using LQR computed gain
                u = -K @ x

                # Extract scalar control input
                command = float(u[0, 0]) * scale_factor

            current_time = time.time()
            if int(current_time * 10) % 1 == 0:
                print(f"Current IMU Pitch Angle: {pitch_angle_deg:.2f} degrees, LQR Output (command): {command:.2f}")

            try:
                await controller_left.set_position(
                    position=math.nan,
                    velocity=command,
                    velocity_limit=1.2,
                    maximum_torque=1,
                    kp_scale=1,
                    kd_scale=12
                )
                await controller_right.set_position(
                    position=math.nan,
                    velocity=-command,
                    velocity_limit=1.2,
                    maximum_torque=1,
                    kp_scale=1,
                    kd_scale=12
                )
            except Exception as e:
                print(f"Error sending commands to motors: {e}")
                continue

    except KeyboardInterrupt:
        print("Stopping robot...")
        try:
            await controller_left.set_stop()
            await controller_right.set_stop()
        except Exception as e:
            print(f"Error stopping motors: {e}")
        ipcon.disconnect()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        ipcon.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
