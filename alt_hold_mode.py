from __future__ import print_function
from math import radians
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

print("Start")

# Подключение к SITL симулятору
connection_string = 'tcp:127.0.0.1:5763'

print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)

point_A = LocationGlobalRelative(50.450739, 30.461242, 50)  # Точка А
point_B = LocationGlobalRelative(50.443326, 30.448078, 100)  # Точка Б


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.armed = True
    print('_____', vehicle.mode.name)

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Altitude: ", vehicle.attitude.yaw)
        print(" Waiting for arming...")
        time.sleep(1)

    print(" Altitude_alt: ", vehicle.location.global_relative_frame.alt)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt

        if current_altitude <= aTargetAltitude - 3:
            vehicle.channels.overrides = {'3': 1900}
            print("Altitude <= 7: ", current_altitude)
        elif current_altitude >= aTargetAltitude - 0.2:
            vehicle.channels.overrides = {'2': 1000, '3': 1500}
            print("Altitude >= 9.8: ", current_altitude)
            print("GO", current_altitude)
        else:
            vehicle.channels.overrides = {'3': 1700}
            print("Stabilizing altitude: ", current_altitude)

        if current_altitude >= aTargetAltitude - 0.2:
            print("Reached target altitude")
            break

        time.sleep(1)


def set_alt_hold_mode():
    """
    Set the vehicle mode to ALT_HOLD.
    """
    print("Setting ALT_HOLD mode")
    vehicle.mode = VehicleMode("ALT_HOLD")
    while vehicle.mode.name != "ALT_HOLD":
        print(" Waiting for ALT_HOLD mode...")
        time.sleep(1)
    print("ALT_HOLD mode set")

def calculate_control_values(target_location):
    current_location = vehicle.location.global_relative_frame
    current_attitude = vehicle.attitude

    distance_to_target = current_location.distance_to(target_location)

    target_heading = current_location.heading_to(target_location)
    relative_heading = target_heading - current_attitude.yaw

    if relative_heading > 180:
        relative_heading -= 360
    elif relative_heading < -180:
        relative_heading += 360

    # Розрахунок pitch, roll і throttle
    pitch = 0 
    roll = 0   
    if distance_to_target > 5:
        throttle = 1600
    else:
        throttle = 1500

    return pitch, roll, throttle

def move_to_point(target_location):
    print("Moving to point with joystick override")
    while vehicle.location.global_relative_frame.get_distance(target_location) > 1:
        pitch, roll, throttle = calculate_control_values(target_location)

        vehicle.channels.overrides = {'1': pitch, '2': roll, '3': throttle}
        time.sleep(0.1)

    print("Reached target location")


def turn_to_heading(target_heading):
    """
    Turn the vehicle to the specified heading.
    """
    current_heading = vehicle.attitude.yaw
    while abs(current_heading - target_heading) > 2:
        vehicle.channels.overrides = {'4': 1300}
        current_heading = vehicle.attitude.yaw
        time.sleep(0.1)
    vehicle.channels.overrides = {}
    print("Reached target heading")


def arm_and_takeoff_and_goto(aTargetAltitude, target_location):
    arm_and_takeoff(aTargetAltitude)
    set_alt_hold_mode()
    move_to_point(target_location)
    print("Turning to yaw 350 degrees")
    turn_to_heading(350)


# Виклик функції:
arm_and_takeoff_and_goto(100, point_B)
