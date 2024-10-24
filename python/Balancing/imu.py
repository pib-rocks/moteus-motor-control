import asyncio
from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_imu_v3 import BrickletIMUV3
import time

HOST = "localhost"
PORT = 4223
UID = "2bS8"  # Replace with your UID

async def main():
    ipcon = IPConnection()
    imu = BrickletIMUV3(UID, ipcon)
    
    try:
        ipcon.connect(HOST, PORT)  # Connect to brickd
        print("Connected to IMU")
        
        while True:
            heading, roll, pitch = imu.get_orientation()
            heading=heading/100
            roll=roll/100
            pitch = (pitch / 100.0)*6.3
            current_time = time.time()
            
            if int(current_time * 100) % 1 == 0:
                print(f"Heading: {heading}, Roll: {roll}, Angle: {pitch}")
            
            await asyncio.sleep(0.1)  # Add a small delay to avoid blocking the event loop
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ipcon.disconnect()  # Ensure the connection is closed when done
        print("Stopping robot...")

if __name__ == "__main__":
    asyncio.run(main())
