from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil



connection_string = 'tcp:127.0.0.1:5763'



print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True


    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(20)
time.sleep(20)


print("Going towards second point for 120 seconds (groundspeed set to 10 m/s) ...")
point2 = LocationGlobalRelative(50.443326, 30.448078, 20)

def fly_to_point_b(point2):
    print('Я уже в функции полета')
    if point2 is None:
        print("No target point provided.")
        return

    print("Going towards second point in ALT_HOLD mode...")
    vehicle.mode = VehicleMode("ALT_HOLD")

    while vehicle.location.global_relative_frame.alt < 9.5:
        print("Waiting for stable altitude...")
        time.sleep(1)


    vehicle.channels.overrides = {'1': 1000, '3': 1700}
    time.sleep(100)

    while not vehicle.mode.name == 'ALT_HOLD':
        print("Waiting for ALT_HOLD mode...")
        time.sleep(1)

fly_to_point_b(LocationGlobalRelative(50.443326, 30.448078, 10))

time.sleep(20)
while True:
    vehicle.channels.overrides = {'4': 1600}
    print(" Altitude: ", vehicle.attitude.yaw)

    if vehicle.attitude.yaw >= 350:
        vehicle.channels.overrides = {'4': 1500}
        print("Reached target altitude___")
        break
    time.sleep(0.3)

"""def set_yaw(angle):
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
        radians(angle), 
        0, 1, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

yaw_angle = 350
set_yaw(yaw_angle)"""

time.sleep(20)
print("Close vehicle object")
vehicle.close()
