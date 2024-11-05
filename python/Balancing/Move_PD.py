import asyncio
import time
import threading
from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_imu_v3 import BrickletIMUV3
from moteus import Controller
from simple_pid import PID
import math

HOST = "localhost"
PORT = 4223
UID = "2bS8"  # Replace UID

# Movement command variables
desired_forward_speed = 0.0
desired_turn_rate = 0.0

def input_thread():
    global desired_forward_speed, desired_turn_rate
    while True:
        command = input("Enter command (w/a/s/d for movement, x to stop): ")
        if command == 'w':
            desired_forward_speed += 0.1
        elif command == 's':
            desired_forward_speed -= 0.1
        elif command == 'a':
            desired_turn_rate += 0.1
        elif command == 'd':
            desired_turn_rate -= 0.1
        elif command == 'x':
            desired_forward_speed = 0.0
            desired_turn_rate = 0.0

async def main():
    ipcon = IPConnection()
    imu = BrickletIMUV3(UID, ipcon)
    ipcon.connect(HOST, PORT)

    # Initialize controllers
    controller_left = Controller(id=1)
    controller_right = Controller(id=2)

    # PID gains, fine tuning needed
    Kp = 0.015  
    Ki = 0
    Kd = 0.0015

    sample_time = 0.005  # 5 ms loop time
    pid = PID(Kp, Ki, Kd, setpoint=0.0)
    pid.sample_time = sample_time

    # Motor limits
    velocity_limit = 1.0        # Adjust as needed
    maximum_torque = 2.0        # Adjust as needed
    kp_scale = 1.0              # Adjust as needed
    kd_scale = 12.0             # Adjust as needed

    # Start the input thread
    threading.Thread(target=input_thread, daemon=True).start()

    print("Starting balancing robot with movement control. Press Ctrl+C to exit.")

    try:
        while True:
            # Read IMU orientation (pitch angle)
            try:
                _, _, pitch = imu.get_orientation()
                pitch_degrees = (pitch / 100.0)*6.3
                current_angle = math.radians(pitch_degrees)
            except Exception as e:
                print(f"Error reading IMU data: {e}")
                continue

            # PID control for balancing
            control = pid(current_angle)

            # Compute wheel velocities
            left_wheel_velocity = control + desired_forward_speed - desired_turn_rate
            right_wheel_velocity = control + desired_forward_speed + desired_turn_rate

            # Limit wheel velocities
            max_velocity = 10.0  # Maximum allowable velocity
            left_wheel_velocity = max(min(left_wheel_velocity, max_velocity), -max_velocity)
            right_wheel_velocity = max(min(right_wheel_velocity, max_velocity), -max_velocity)

            # Send velocity commands to the motors
            try:
                await controller_left.set_position(
                    position=math.nan,
                    velocity=left_wheel_velocity,
                    velocity_limit=velocity_limit,
                    maximum_torque=maximum_torque,
                    kp_scale=kp_scale,
                    kd_scale=kd_scale
                )
                await controller_right.set_position(
                    position=math.nan,
                    velocity=right_wheel_velocity,
                    velocity_limit=velocity_limit,
                    maximum_torque=maximum_torque,
                    kp_scale=kp_scale,
                    kd_scale=kd_scale
                )
            except Exception as e:
                print(f"Error sending commands to motors: {e}")
                continue

            # Sleep to maintain loop timing
            await asyncio.sleep(sample_time)

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
