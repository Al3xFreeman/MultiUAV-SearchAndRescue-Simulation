import dronekit as dk
import dronekit_sitl as dk_sitl
import time

from funciones import *

#COmprobar antes si se le ha mandado connection String, si 
#Inicia el simulador
sim = IniciaSITL()




controladores = []
#Ver cómo va lo de instance_count de dronekit_sitl
print("CON: ", sim.getConnectionString())
controladores.append(ControladorDron(sim.getConnectionString()))
sim.sitl.instance += 1
print("INSTANCIA: ", sim.sitl.instance)
print("CON: ", sim.getConnectionString())
#controladores.append(ControladorDron(sim.getConnectionString()))
#sim.sitl.instance += 1
#print("INSTANCIA: ", sim.sitl.instance)

points = controladores[0].generateRoute(file = "puntosPoligono.txt", granularity = 10)

print("LOCATIONS RETURN")
for p in points:
    print(p)

#controlador.despega(controlador.vehicle, 10)
controladores[0].despega(20)


controladores[0].executeMission(points)
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


#controlador.recorreArea("puntosPoligonos.txt")



#Posteriormente investigar el tema de las misiones
#https://dronekit-python.readthedocs.io/en/latest/guide/auto_mode.html
#https://dronekit-python.readthedocs.io/en/latest/examples/mission_basic.html#example-mission-basic

#Después hacer movimiento automático por una zona designada y que lo recorra entero
