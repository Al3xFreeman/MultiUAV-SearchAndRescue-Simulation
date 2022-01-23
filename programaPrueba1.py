import dronekit as dk
import dronekit_sitl as dk_sitl
import time

from funciones import *

controlador = ControladorDron()

controlador.generateRoute(file = "puntosPoligono.txt", granularity = 25)

#controlador.despega(controlador.vehicle, 10)
controlador.despega(40)


controlador.executeMission()
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
