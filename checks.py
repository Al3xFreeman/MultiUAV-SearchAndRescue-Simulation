import dronekit as dk
import dronekit_sitl as dk_sitl
import time



def pre_arm(vehicle :dk.Vehicle):
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


def arming(vehicle :dk.Vehicle):
    print("Arming motors")
    # Copter should arm in GUIDED mode
    print (vehicle.mode.name)
    vehicle.mode =dk.VehicleMode("GUIDED")

    vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    

def esperaAltura(vehicle:dk.Vehicle, altura):
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= altura * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

