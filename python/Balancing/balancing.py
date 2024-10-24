import asyncio
import time
from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_imu_v3 import BrickletIMUV3
from moteus import Controller
from simple_pid import PID
import math

HOST = "localhost"
PORT = 4223
UID = "2bS8"  #Replace UID

async def main():
    ipcon = IPConnection()
    imu = BrickletIMUV3(UID, ipcon)
    ipcon.connect(HOST, PORT)

    #Initialize controllers
    controller_left = Controller(id=1)
    controller_right = Controller(id=2)

    # PID gains, fine tuning needed
    Kp = 0.015  
    Ki = 0
    Kd = 0.0015

    sample_time = 0.005   # 10 ms loop time
    pid = PID(Kp, Ki, Kd, setpoint=0.0)
    pid.sample_time = sample_time


    print("Starting balancing robot. Press Ctrl+C to exit.")

    try:
        while True:
            # Read IMU orientation (pitch angle)
            try:
                _, _, pitch = imu.get_orientation()
                current_angle = (pitch/100.0)*6.3
            except Exception as e:
                print(f"Error reading IMU data: {e}")
                continue

            angle_error = 0 - current_angle
            deadband = 0.2 
            # Apply deadband to prevent excessive corrective actions
            if abs(angle_error) < deadband:
                control = 0.0
            else:
                control = pid(current_angle)

            current_time = time.time()
            if int(current_time * 10) % 1 == 0:
               print(f"Current IMU Pitch Angle: {current_angle:.2f} degrees, PID Output (velocity): {control:.2f}")
    
            # Send velocity commands to the motors
            try:
                await controller_left.set_position(
                    position=math.nan,  
                    velocity=control,
                    velocity_limit=1,
                    maximum_torque=1,
                    kp_scale=1,
                    kd_scale=12
                )
                await controller_right.set_position(
                    position=math.nan,     
                    velocity=control,
                    velocity_limit=1,
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
