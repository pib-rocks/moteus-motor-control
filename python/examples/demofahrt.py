#! /usr/bin/python3
#

import sys
import asyncio
import math
import moteus
import argparse
import time

def custom_usage():
    return f'''Usage: python {sys.argv[0]} [options] <mode> <time> <velocity>

Mandatory arguments:
  mode        Specify the mode, one of: "demo", "forward", "backward", "right", "left", "init"
              (first letter is sufficient)
  time        Specify the duration in seconds (float)
  velocity    Specify the velocity in revolutions per sec (float)


Optional arguments:
  --no-synchronize    Disable synchronization
  -v, --verbose       Increase output verbosity
'''

parser = argparse.ArgumentParser(usage=custom_usage(), add_help=False)

# Optional arguments
parser.add_argument('--no-synchronize', action='store_true')
parser.add_argument('--verbose', '-v', action='store_true')

# Mandatory arguments
parser.add_argument('mode', type=str, help='Specify the mode')
parser.add_argument('time', type=float, help='Specify the time')
parser.add_argument('velocity', type=float, help='Specify the velocity')

#Code copied from Moteus example "synchronized movement"
args = parser.parse_args()

#Number of motors
SERVO_IDS = [1,2]

qr = moteus.QueryResolution()
qr.trajectory_complete = moteus.INT8

# Generate controller
controllers = {x: moteus.Controller(x, query_resolution=qr) for x in SERVO_IDS}
# Generate streams (indexed by their espective controller) for commands
streams = {y: moteus.Stream(y) for y in controllers.values()}

#Code copied from Moteus example "synchronized movement"
def _calculate_ms_delta(time1, time2):
    # These are returned as int32s, so they may have wrapped around.
    if time2 < 0 and time1 > 0:
        result_ms = time2 + (2**32) - time1
    else:
        result_ms = time2 - time1
    if result_ms > 100000 or time2 < time1:
        # We'll assume any difference of more than 100s is a problem
        # (or a negative difference).
        return None
    return result_ms

#Code copied from Moteus example "synchronized movement"
class ServoClock:
    '''This class can be used to keep a controller's time base
    synchronized with the host time base.

    Instantiate it, then call await ServoClock.update_second() at a
    regular rate, approximately at 1Hz.
    '''

    # This constant should be approximately 5-10x the update rate.
    # Smaller means that the device time will converge with the host
    # time more quickly, but will also be less stable.
    TIME_CONSTANT = 5.0

    # This is the approximate change in clock rate of the device for
    # each trim count change.
    TRIM_STEP_SIZE = 0.0025

    def __init__(self, controller, measure_only=False):
        self.controller = controller
        self.initial_time = None
        self.state = None
        self.device_ms_count = 0
        self.measure_only = measure_only

        # This is the currently reported time error between the host
        # and the device.
        self.time_error_s = 0.0

        self._query = self.controller.make_custom_query(
            {moteus.Register.MILLISECOND_COUNTER : moteus.INT32,
             moteus.Register.CLOCK_TRIM: moteus.INT32}
        )

    async def update_second(self):
        '''This should be called at a regular interval, no more often than
        once per second.
        '''

        print("ServoClock.update_second()")

        old_state = self.state

        self.state = await self.controller.execute(self._query)

        now = time.time()

        ms_count_delta = 0

        if old_state is not None:
            ms_count_delta = _calculate_ms_delta(
                old_state.values[moteus.Register.MILLISECOND_COUNTER],
                self.state.values[moteus.Register.MILLISECOND_COUNTER])

            if ms_count_delta is None:
                # We became desynchronized and need to restart.
                self.device_ms_count = 0
                self.initial_time = None
            else:
                self.device_ms_count += ms_count_delta

        if self.initial_time is not None and ms_count_delta != 0:
            # We have enough information to calculate an update.

            # First, we calculate the delta between our starting time
            # and now in the host timespace.
            total_host_time = now - self.initial_time

            # And the amount of host time that has advanced since our
            # last call.  This should be approximately 1s.
            period_host_time = now - self.last_time

            # Now measure the absolute drift in seconds between the
            # device and our host clock.
            absolute_drift = self.device_ms_count / 1000 - total_host_time
            self.time_error_s = absolute_drift

            # And secondarily measure the ratio between the device
            # time and host time during the last period.
            rate_drift = (ms_count_delta / 1000) / period_host_time

            # What drift would we need to cancel out our total
            # absolute drift over the next TIME_CONSTANT seconds?
            desired_drift = 1 + -absolute_drift / self.TIME_CONSTANT

            # Figure out how much we need to change the devices clock
            # rate in order to cancel that drift, both as a floating
            # point value, then again in integer counts.
            delta_drift = desired_drift - rate_drift
            delta_drift_integral = round(delta_drift / self.TRIM_STEP_SIZE)

            # Finally, we figure out the new trim value we should ask
            # for.
            new_trim = (self.state.values[moteus.Register.CLOCK_TRIM] +
                        delta_drift_integral)

            if not self.measure_only:
                await self.controller.set_trim(trim=new_trim)

        if self.initial_time is None:
            self.initial_time = now

        self.last_time = now

#Code copied from Moteus example "synchronized movement"
class Poller:
    TIMESTAMP_S = 0.01
    CLOCK_UPDATE_S = 1.0

    def __init__(self, controllers, args):
        self.controllers = controllers
        self.last_time = time.time()
        self.servo_data = {}
        self.servo_clocks = {
            x.id: ServoClock(x, measure_only=args.no_synchronize)
            for x in controllers.values()
        }

        self.next_clock_time = self.last_time + self.CLOCK_UPDATE_S


    async def wait_for_event(self, condition, timeout=None, per_cycle=None):
        start = time.time()
        while True:
            now = time.time()
            if (now - start) > timeout:
                raise RuntimeError("timeout")

            if now > self.next_clock_time:
                self.next_clock_time += self.CLOCK_UPDATE_S
                [await x.update_second() for x in self.servo_clocks.values()]

            self.last_time += self.TIMESTAMP_S
            delta_s = self.last_time - now
            await asyncio.sleep(delta_s)

            self.servo_data = {x.id : await x.query()
                               for x in self.controllers.values()}

            if per_cycle:
                per_cycle()

            if condition():
                return

#Check modus and call correct function
async def main(modus, time, v):

    # Initialization of the motors without moving.
    await initialize(0)
    if modus[0]=="d":
        print("Demo-drive")
        await demo_drive(v)
    elif modus[0]=="f":
        print("Modus forward")
        await forward(time, v)
    elif modus[0]=="b":
        print("Modus backward")
        await backward(time, v)
    elif modus[0]=="r":
        print("Modus right")
        await right(time, v)
    elif modus[0]=="l":
        print("Modus left")
        await left(time, v)
    elif modus[0]=="i":
        # full initialization of the motors. They will be set to position 0
        await initialize(1)
    else:
        print(f"ERROR: unknown mode %{modus}. Try {parser.prog} --help")
        sys.exit(1)

async def initialize(full):
    #remove validation of positions
    print("maximum position set to NaN")
    for c in streams: await streams[c].command(b"conf set servopos.position_min NaN")
    print("minimum position set to NaN")
    for c in streams: await streams[c].command(b"conf set servopos.position_max NaN")

    if not full: return

    print("Attention: Initalizing wheels")
    await asyncio.sleep(1)

    poller = Poller(controllers, args)

    # Tell all the servos to go to position 0.0.
    for servo in controllers.values():
        await servo.set_position(position=0.0, velocity_limit=0.5, accel_limit=5.0, watchdog_timeout=math.nan)

    await poller.wait_for_event(
        lambda: all([x.values[moteus.Register.TRAJECTORY_COMPLETE]
                     for x in poller.servo_data.values()]), timeout=5.0)
    print("Initilazation completed")


async def demo_drive(v):
    #Demo-Program
    await forward(5, v)
    await right(5, v)
    await backward(5, v)
    await left(5, v)
    await forward(5, v)

async def forward(time, v):
    print("drive forward")
    # Tell all the servos to go forward with given time in sec and velocity v in rotations per second
    [await asyncio.wait_for(servo.set_position(
        position=math.nan, velocity=v, accel_limit=5.0, watchdog_timeout=math.nan), timeout=time)
     for servo in controllers.values()]

    poller = Poller(controllers, args)
    await poller.wait_for_event(
        lambda: all([x.values[moteus.Register.TRAJECTORY_COMPLETE]
                     for x in poller.servo_data.values()]), timeout=5.0)


async def backward(time, v):
    print("drive backward")
    # Tell all the servos to go backward with given time in sec and velocity v in rotations per second
    [await servo.set_position(
        position=math.nan, velocity=v*(-1), accel_limit=5.0, watchdog_timeout=math.nan)
     for servo in controllers.values()]


async def right(time, v):
    print("drive right")
    # Tell servo 1 to go with given time in sec and velocity v in rotations per second
    servo = controllers[1]
    await servo.set_position(position=math.nan, velocity=v, accel_limit=5.0, watchdog_timeout=time)

async def left(time, v):
    print("drive left")
    # Tell servo 2 to go with given time in sec and velocity v in rotations per second
    servo = controllers[2]
    await servo.set_position(position=math.nan, velocity=v, accel_limit=5.0, watchdog_timeout=time)

if __name__ == '__main__':

        asyncio.run(main(args.mode, args.time, args.velocity))

