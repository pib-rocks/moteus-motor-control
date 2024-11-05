import asyncio
import time
from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_imu_v3 import BrickletIMUV3
from moteus import Controller
import math
import numpy as np
import control as ct
import threading
import sys

# For keyboard input
import tty
import termios

HOST = "localhost"
PORT = 4223
UID = "2bS8"  # Replace UID

# Global variable for user input
user_velocity = 0.0
user_turn = 0.0

def keyboard_input():
    global user_velocity, user_turn
    # Set terminal to raw mode
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)
    
    try:
        while True:
            key = sys.stdin.read(1)
            if key == 'w':
                user_velocity += 0.1  # Increase forward speed
            elif key == 's':
                user_velocity -= 0.1  # Decrease forward speed
            elif key == 'a':
                user_turn += 0.1      # Turn left
            elif key == 'd':
                user_turn -= 0.1      # Turn right
            elif key == 'q':
                break  # Exit the loop on 'q' key
            else:
                pass  # Ignore other keys
            # Limit the user_velocity and user_turn
            user_velocity = max(min(user_velocity, 1.0), -1.0)
            user_turn = max(min(user_turn, 0.5), -0.5)
    finally:
        # Restore terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

async def main():
    global user_velocity, user_turn
    ipcon = IPConnection()
    imu = BrickletIMUV3(UID, ipcon)
    ipcon.connect(HOST, PORT)

    # Initialize controllers
    controller_left = Controller(id=1)
    controller_right = Controller(id=2)

    # System parameters for inverted pendulum on cart
    M = 0.5   # Mass of the cart (kg)
    m = 0.2   # Mass of the pendulum (kg)
    b = 0.1   # Coefficient of friction for cart (N/m/sec)
    I = m * l**2 # Mass moment of inertia of the pendulum (kg*m^2)
    g = 9.81  # Gravity acceleration (m/s^2)
    l = 0.3   # Length to pendulum center of mass (m)

    # Denominator for the A and B matrices
    denom = I*(M+m) + M*m*l**2

    # State-space representation
    A = np.array([
        [0, 1, 0, 0],
        [0, -(I + m * l**2) * b / denom, (m**2 * g * l**2) / denom, 0],
        [0, 0, 0, 1],
        [0, -(m * l * b) / denom, m * g * l * (M + m) / denom, 0]
    ])

    B = np.array([
        [0],
        [(I + m * l**2) / denom],
        [0],
        [m * l / denom]
    ])

    # LQR weighting matrices
    Q = np.diag([10, 1, 10, 1])
    R = np.array([[0.1]])

    # Compute LQR gain
    K, _, _ = ct.lqr(A, B, Q, R)

    # Scale factor to map control input to motor command
    scale_factor = 0.05  # Adjust as necessary

    # Deadband thresholds
    deadband_angle_deg = 0.5  # degrees

    # Start keyboard input thread
    input_thread = threading.Thread(target=keyboard_input)
    input_thread.daemon = True
    input_thread.start()

    print("Starting balancing robot with LQR control and keyboard input. Press 'q' to exit.")

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

                # Assume x and x_dot can be approximated from wheel encoders
                # For simplicity, we set x and x_dot to zero (you can implement odometry here)
                x_pos = 0.0
                x_dot = (controller_left.state.velocity + controller_right.state.velocity) / 2.0
            except Exception as e:
                print(f"Error reading sensors: {e}")
                continue

            # Form state vector x
            x = np.array([
                [x_pos],
                [x_dot],
                [pitch_angle_rad],
                [pitch_rate_rad_per_sec]
            ])

            # Desired state (setpoints)
            x_desired = np.array([
                [0.0],                # Desired position (you can set this to control position)
                [user_velocity],      # Desired velocity (from user input)
                [0.0],                # Desired pitch angle (upright)
                [0.0]                 # Desired pitch rate
            ])

            # Compute control input u using LQR
            u = -K @ (x - x_desired)

            # Extract scalar control input
            command = float(u[0, 0]) * scale_factor


            # Adjust for turning
            left_command = command + user_turn
            right_command = command - user_turn

            # Logging for debugging
            current_time = time.time()
            if int(current_time * 10) % 1 == 0:
                print(f"Pitch Angle: {pitch_angle_deg:.2f}°, Command: {command:.2f}, Left Cmd: {left_command:.2f}, Right Cmd: {right_command:.2f}")

            # Send velocity commands to the motors
            try:
                await controller_left.set_position(
                    position=math.nan,
                    velocity=left_command,
                    velocity_limit=2,
                    maximum_torque=1,
                    kp_scale=1,
                    kd_scale=12
                )
                await controller_right.set_position(
                    position=math.nan,
                    velocity=right_command,
                    velocity_limit=2,
                    maximum_torque=1,
                    kp_scale=1,
                    kd_scale=12
                )
            except Exception as e:
                print(f"Error sending commands to motors: {e}")
                continue

            # Small delay to prevent high CPU usage
            await asyncio.sleep(0.01)

    except KeyboardInterrupt:
        print("Stopping robot...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        try:
            await controller_left.set_stop()
            await controller_right.set_stop()
        except Exception as e:
            print(f"Error stopping motors: {e}")
        ipcon.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
