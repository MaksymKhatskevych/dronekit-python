from __future__ import print_function
from math import radians
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil

print("Start")
# Подключение к SITL симулятору
connection_string = 'tcp:127.0.0.1:5763'

print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and flies to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(10)

# Увеличьте время взлета
time.sleep(30)

print("Going towards second point for 120 seconds (groundspeed set to 10 m/s) ...")
point_A = LocationGlobalRelative(50.450739, 30.461242, 10)
point_B = LocationGlobalRelative(50.443326, 30.448078, 10)

def fly_to_point_b(point_B):
    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.channels.overrides['3'] = 10

    # Ждем, пока БЛА достигнет стабильной высоты
    while vehicle.location.global_relative_frame.alt < 9.5:
        print("Waiting for stable altitude...")
        time.sleep(1)

    msg = vehicle.message_factory.set_position_target_global_int_encode(
        120,  # time_boot_ms
        0,  # target_system
        0,  # target_component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111111000,  # type_mask (ignore all except position bits)
        int(point_B.lat * 1e7),  # lat (degrees * 1e7)
        int(point_B.lon * 1e7),  # lon (degrees * 1e7)
        point_B.alt,  # alt (meters)
        10,  # vx (m/s)
        10,  # vy (m/s)
        10,  # vz (m/s)
        0,  # afx (N)
        0,  # afy (N)
        0,  # afz (N)
        0,  # yaw (degrees * 100)
        0,  # yaw_rate (degrees/s * 100)
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

fly_to_point_b(point_B)

# Добавьте небольшую задержку перед отправкой MAVLink-сообщения
time.sleep(2)

# Ждем, пока БЛА достигнет точки B
while not vehicle.mode.name == 'ALT_HOLD':
    print("Waiting for ALT_HOLD mode...")
    time.sleep(1)

while not vehicle.location.global_relative_frame.distance_to(point_B) < 1:
    print("Distance to target: ", vehicle.location.global_relative_frame.distance_to(point_B))
    time.sleep(1)
# После прибытия на точку B поворачиваемся
def set_yaw(angle):
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
        radians(angle),  # Угол задается в радианах
        0, 1, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

yaw_angle = 350
set_yaw(yaw_angle)

time.sleep(20)
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.