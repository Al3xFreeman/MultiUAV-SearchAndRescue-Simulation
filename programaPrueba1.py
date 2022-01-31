from statistics import mode
import dronekit as dk
import dronekit_sitl as dk_sitl
import time

from funciones import *
from generadorRutas import *

from typing import List



#COmprobar antes si se le ha mandado connection String, si 
#Inicia el simulador
sim = IniciaSITL()

num_drones = 1
controladores : List[ControladorDron] = [] 


modo = Modos.Single

generadorRutas = GeneraRutas(file = "puntosPoligono.txt", granularity=25, modo=modo)
rutas = generadorRutas.generaRuta()


for i in range(num_drones):
    #Ver cómo va lo de instance_count de dronekit_sitl
    print("CON: ", sim.getConnectionString())
    controladores.append(ControladorDron(sim.getConnectionString()))
    sim.sitl.instance += 1
    print("INSTANCIA: ", sim.sitl.instance)

#Ver una forma de poder crear un archivo de la ruta en lugar de pasar los objetos de las posiciones a recorrer
controladores[0].createMission(rutas[0])
controladores[0].recorreArea(rutas[0])
#Separar la generación de la ruta del funcionamiento del dron
#Primero generar la ruta y luego ocnectar los drones.
#   Si no, lo más probable es que se desconecten porque no reciban ningún mensaje.
#Mandar a cada dron la parte de la ruta que debe realizar.
#Cada dron deberá avisar cuando acabe una ruta asignada para que se le pueda asignar la siguiente.

#Varios modos:
#   - Single route: Una sola ruta que se divide en n trozos, siendo n el número de drones.
#   - mTSP: Problema del viajante pero con varios vehículos, siendo n el número de drones.
#   - Sectores: Dividir el área en sectores con el mísmo número de puntos (un número computable)
#                y asignar a los drones los distintos sectores

#Ver la diferencia de cómputo entre los 3 modos:
#   - Single: Si son muchos puntos siempre va a ser problemátio.
#   - mTSP: Mismo problema
#   - Sectores: Tal vez tarda mucho en seccionar el área en n sub-áreas?

controladores[0].recorreArea(file = "puntosPoligono.txt", granularity = 10)

#points = controladores[0].generateRoute(file = "puntosPoligono.txt", granularity = 10)
#controladores[0].executeMission(points)

#Primero hacer un sistema para mover el dron con un input por teclado (o incluso con el mando?)
#https://stackoverflow.com/questions/46506850/how-can-i-get-input-from-an-xbox-one-controller-in-python


#Establecer un área (de momento hagamos que sea un cuadrilátero)
# Generar los waypoints necesarios para recorrer toda esa zona
# Buscar alguna forma de saber la separacióm entre ida y venida (cuando recorre la zona, básicamente)
# Es decir, ver cómo lo hace el Mission Planner por ejemplo.

# Para un polígono cualquiera:
# 1) Comprobar que no se intersecta a si mismo para que sea válido
# 2) Generar la matriz de puntos (parametrizar la granularidad de la matriz en función de la altura a la que vaya a volar el dron)
#        A más altura, más alejados estarán los puntos.
# 3) Ver cuales están dentro del polígono
# 4.1) Ejecutar TSP con los que estén dentro (el primer punto es donde esté el dron al iniciar)
# 4.2) Si el punto inicial no es donde está el dron, parametrizarlo y que el dron vaya a ese punto inicial
# 5.1) Con la ruta que devuelve el TSP, hacer la lista de Waypoints y subírselos al dron.
# 5.2) Si el punto inicial NO era donde estba el dron el RTL debería volver al inicio del todo.


#Posteriormente investigar el tema de las misiones
#https://dronekit-python.readthedocs.io/en/latest/guide/auto_mode.html
#https://dronekit-python.readthedocs.io/en/latest/examples/mission_basic.html#example-mission-basic

#Después hacer movimiento automático por una zona designada y que lo recorra entero
