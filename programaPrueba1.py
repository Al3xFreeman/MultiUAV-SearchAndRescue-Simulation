import dronekit as dk
import dronekit_sitl as dk_sitl
import time

from funciones import *

controlador = ControladorDron()

controlador.despega(controlador.vehicle, 10)



#Primero hacer un sistema para mover el dron con un input por teclado

#Posteriormente investigar el tema de las misiones

#Después hacer movimiento automático por una zona designada y que lo recorra entero