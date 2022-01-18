from time import sleep
import dronekit as dk
import dronekit_sitl as dk_sitl


from funciones import *

controlador = ControladorDron()

print(controlador.vehicle.mode.name)

controlador.vehicle.mode = dk.VehicleMode("GUIDED")
sleep(2)
print(controlador.vehicle.mode.name)