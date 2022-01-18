from dronekit import Command, LocationGlobal, VehicleMode
from funciones import *
import os
from pymavlink import mavutil


controlador = ControladorDron()

script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
rel_path = "rutas/ruta1.txt"
abs_file_path = os.path.join(script_dir, rel_path)
print(abs_file_path)
with open(abs_file_path) as file:
    lines = file.readlines()
    lines = [line.rstrip() for line in lines]

rel = lines[0] == "rel" #Opcion de poner los valores globales
path = list(map(lambda x: list(map(float, x.split(","))), lines[1:]))
startPath = path[0]
a_location = LocationGlobal(startPath[0], startPath[1], controlador.vehicle.location.global_frame.alt)
print("Locations parsed")

startPos = controlador.vehicle.location.global_frame

#commands = controlador.vehicle.commands
controlador.vehicle.commands.clear()

#Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
controlador.vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
print("Command takeoff")

puntos = []
puntos.append(a_location)
controlador.vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, a_location.lat, a_location.lon, 11))
print("Inicio")

prev = a_location
print(a_location.lat, a_location.lon)
for elem in path[2:]:
    
    punto = get_location_metres(prev, elem[0], elem[1])
    print(punto)
    puntos.append(punto)

    controlador.vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, punto.lat, punto.lon, 11))
    print("Añadido", len(controlador.vehicle.commands))
    prev = punto

print(puntos[0].lat, puntos[0].lon)

print("Upload")

controlador.vehicle.commands.upload()

print("Uploaded")

controlador.despega(10)

# Reset mission set to first (0) waypoint
controlador.vehicle.commands.next=0

# Set mode to AUTO to start mission
controlador.vehicle.mode = VehicleMode("AUTO")


print(path)

print ("Global Location (relative altitude): %s" % controlador.vehicle.location.global_relative_frame)
print ("Altitude relative to home_location: %s" % controlador.vehicle.location.global_relative_frame.alt)

# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.

while True:
    nextwaypoint=controlador.vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextwaypoint, controlador.distance_to_current_waypoint()))
    if nextwaypoint==len(puntos) + 1: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
        print("Exit 'standard' mission when start heading to final waypoint (5)")
        break;
    time.sleep(1)

print('Return to launch')
controlador.vehicle.mode = VehicleMode("RTL")


#Close vehicle object before exiting script
print("Close vehicle object")
controlador.vehicle.close()

