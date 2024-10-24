import asyncio
import moteus
import math

# Parameters to set
parameters = {
    'servo.derate_temperature': 80.0,
    'servo.fault_temperature': 85.0,
    'servo.pid_position.kp': 80.0,
    'servo.pid_position.ki': 0.0,
    'servo.pid_position.kd': 0.2,
    'servopos.position_min': math.nan,
    'servopos.position_max': math.nan,

}

async def set_parameters(controller, params):
    stream = moteus.Stream(controller)
    await stream.flush_read()
    await stream.command(b'd stop')

    for param, value in params.items():
        command = f'conf set {param} {value}'.encode('latin1')
        await stream.command(command)
    # Optionally, save the configuration to flash memory
    await stream.command(b'conf write')
    print(f"Parameters set for controller ID {controller.id}")

async def main():
    # Initialize Moteus motor controllers
    controller_left = moteus.Controller(id=1)
    controller_right = moteus.Controller(id=2)
    
    try:
        print("Setting parameters on Moteus r4...")
        await set_parameters(controller_left, parameters)
        await set_parameters(controller_right, parameters)
        print("Parameters successfully set on both motors.")
    except Exception as e:
        print(f"An error occurred: {e}")

# Run the asynchronous main function
asyncio.run(main())
