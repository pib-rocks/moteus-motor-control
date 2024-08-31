#! /usr/bin/python3
#
# References:
#       https://github.com/mjbots/moteus/blob/main/docs/reference.md#velocity-control

import sys
import asyncio
import math
import moteus
import argparse
import time
from numpy import isnan

def custom_usage():
    return f'''Usage: python {sys.argv[0]} [options] <right> <left> <time> <init>

Mandatory arguments:
  right float, revolutions of right motor per seconds. Positive forwards, negative backwards


Optional arguments:
  left float, revolutions of right motor per seconds. Positive forwards, negative backwards. If not provided or nan: Copy value from right
  time float, time in seconds of movement. Default is 3 seconds
  init bool, execute initialization. Default is True 
'''

parser = argparse.ArgumentParser(usage=custom_usage(), add_help=False)

#Optional Arguments
parser.add_argument('--left', type=float, default=math.nan, help='Specify the velocity of left motor')
parser.add_argument('--time', type=float, default=3.0, help='Specify the time of movement in seconds')
parser.add_argument('--init', type=bool, default=True, help='Determine if initialization is needed')

#Mandatory Arguments
parser.add_argument('right', type=float, help='Specify the velocity of right motor')


#Code copied from Moteus example "synchronized movement"
args = parser.parse_args()


#Number of motors
SERVO_IDS = [1,2]

qr = moteus.QueryResolution()
qr.trajectory_complete = moteus.INT8

# Generate controller
controllers = {x: moteus.Controller(x, query_resolution=qr) for x in SERVO_IDS}
# Generate streams (indexed by their espective controller) for commands
streams = {y: moteus.Stream(controllers[y]) for y in controllers}


#main function
#   right: velocity in revolutions per sec for the right wheel, ID = 1
#   left: analogue to right. If left is nan, then the right value will be set
#   time: time of movement. If nan, then endless movement
#   init: initialisation of the motors
async def main(right, left=math.nan, time = 3.0, init = True):
    if init: 
        #delete ristrictions for valid motor positions
        for stream in streams.values(): await stream.command(b"conf set servopos.position_min NaN")
        print("minimum position set to NaN")
        for stream in streams.values(): await stream.command(b"conf set servopos.position_max NaN")
        print("maximum position set to NaN")

        #set PID parameters
        for stream in streams.values(): await stream.command(b"conf set servo.pid_position.kp 20")
        for stream in streams.values(): await stream.command(b"conf set servo.pid_position.ki 1")
        for stream in streams.values(): await stream.command(b"conf set servo.pid_position.kd 1")

        #set max parameters
        for stream in streams.values(): await stream.command(b"conf set servo.max_velocity 8")
        for stream in streams.values(): await stream.command(b"conf set servo.fault_temperature 80")
        
        #if timeout is reached, a stop is necessary for further movements
        for stream in streams.values(): await stream.command(b"d stop")

    #If left is not provided (nan) then copy the value from right
    if isnan(left):
        left = right
    
    #move the right motor (d pos is the command, first paratemer position is nan, second parameter is velocity, third is max torque set to 3, v8 is velocity limit 8, a3 is acceleration limit 3 and t is timeout in second given by time)
    servo_1 = streams[1]
    await servo_1.command(b"d pos nan "+str(right).encode()+b" nan v8 a200 t"+str(time).encode())
    #move the left motor
    servo_2 = streams[2]
    await servo_2.command(b"d pos nan "+str(left).encode()+b" nan v8 a200 t"+str(time).encode())

if __name__ == '__main__':
    asyncio.run(main(args.right, args.left, args.time, args.init))
