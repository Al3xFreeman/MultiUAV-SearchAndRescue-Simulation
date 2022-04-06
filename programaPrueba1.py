from statistics import mode
import dronekit as dk
import dronekit_sitl as dk_sitl
import time

from funciones import *
from generadorRutas import *

from typing import List

import math

import threading

from random import randrange

#COmprobar antes si se le ha mandado connection String, si 
#Inicia el simulador


parser = argparse.ArgumentParser(description='Parametros pal programa')
parser.add_argument('-D',
                    '-d',
                    '--drones',
                    help="Cantidad de drones a utilizar",
                    required= False,
                    default=1)
parser.add_argument('-G',
                    '-g',
                    '--granularity',
                    help="granularidad de los puntos a recorrer por los drones",
                    required= False,
                    default=20)
parser.add_argument('-O',
                    '-o',
                    '--output',
                    help="Habilita la salida de monitorización de drones",
                    action='store_true')
parser.add_argument('-A',
                    '-a',
                    '--alert',
                    help="Thread para alertar sobre cuando no están los comandos",
                    action='store_true')

args = parser.parse_args()


num_drones = int(args.drones)
granularity = int(args.granularity)
output = args.output
alert = args.alert

print("NUMERO DE DRONES: ", num_drones)
time.sleep(2)

#controladores : List[ControladorDron] = [] 
sims : List[IniciaSITL] = []
drones : List[ControladorDron] = []

modo = Modos.Single

objetivoDetectado = False

#print("VALOR DE OBJETIVO DETECTADO: ", objetivoDetectado)
posicionObjetivo = None

generadorRutas = GeneraRutas(file = "puntosPoligono.txt", granularity=granularity, modo=modo)
rutas = generadorRutas.generaRuta()

if modo == Modos.Single:
    #Esto sirve para cuando es una sola ruta (Modos.Single) y tenemos multiples drones
    def divideRutaEntreDrones(numDrones, ruta):
        segmentSize = math.floor(len(ruta)/num_drones)
        remainingSpots = len(ruta) - (segmentSize * num_drones)
        spotsPerDrone = [segmentSize] * num_drones
        index = 0
        print("Remaining spots:", remainingSpots)
        while(remainingSpots != 0):
            
            if(index % num_drones == 0):
                index = 0
            print("index: ", index)
            spotsPerDrone[index] += 1
            remainingSpots -= 1
            index += 1
        print("Spots per drone")
        print(spotsPerDrone)
        rutaSegmentadas = []
        desde = 0
        hasta = 0
        for i in range(numDrones):
            hasta += spotsPerDrone[i]
            rutaSegmentadas.append(ruta[desde : hasta])

            desde = hasta

        return rutaSegmentadas
    
    rutasMultDrones = divideRutaEntreDrones(num_drones, rutas[0])
    for i, e in enumerate(rutasMultDrones):
        print("ID: ", i, " | LEN: ", len(e))

def nuevoDron(id, camaraActivada, ruta):
    global soloUnaCamara
    time.sleep(id * 0.2)
    print("INICIANDO DRON ID: ", id)
    sim = IniciaSITL()

    print("SITL INICIADO")

    sims.append(sim)
    #print("CON: ", sim.connection_string)
    
    dron = ControladorDron(sim.connection_string, id, len(ruta), lastPoint=ruta[-1], wayPointLocations=ruta)
    drones.append(dron)
    #controladores.append(ControladorDron(sim.connection_string, id))

    #dron.despega(5)

    dron.vehicle.mode = dk.VehicleMode("AUTO")
    
    print("RUTA DEL DRON con ID:", id, " | LEN: ", len(ruta))

    dron.createMission(ruta)
    
    print("Esperando a que el dron se inicie y se establezca la misión")
    time.sleep(1)
    print("Iniciando dron")
    dron.iniciaDron(camaraActivada)
    #Ver cómo va lo de instance_count de dronekit_sitl
    #sim.sitl.instance += 1
    #print("INSTANCIA: ", sim.sitl.instance)

thread_list_start = []
camaraActivada = True

for i in range(num_drones):
    thread = threading.Thread(target=nuevoDron, args=(i,camaraActivada, rutasMultDrones[i]))
    camaraActivada = False
    thread_list_start.append(thread)

def allDronesFinished(drones : List[ControladorDron]):
    for drone in drones:
        if not drone.finished:
            return False
    
    return True

finishMonitor = False

def monitorDrones():
    global objetivoDetectado
    #Damos tiempo a que se inicien los drones
    time.sleep(30)
    global finishMonitor
    finishMonitor = False
    while(not finishMonitor):
        print()
        print()
        print()
        print()
        print()
        print()
        print()
        print()
        print()
        print()
        print("**************************************************************************")
        print("Obteniendo información sobre drones...")
        print("NUMERO DE DRONES: ", len(drones))
        
        #Comprobación de si algún dron ha encontrado al objetivo
        objetivoDetectado = False
        for dron in drones:
            if(dron.objetivoEncontrado):
                objetivoDetectado = True
                posicionObjetivo = dron.vehicle.location.global_frame

                print("Procediendo a terminar la ejecución del resto de drones")
                for d in drones:
                    d.finished = True


        for dron in drones:
            #TODO: Hacer que la comprobación de si han terminado no dependa de si hay o no drones
            #   Ya que si se están iniciando, peta. Dejarlo como una variable del programa principal que sea controlada por la ejecución de cada dron
            #print("FINISHED: ", dron.finished)
            if(not dron.finished):
                dron.getInfo()
            else:
                print("Dron: ", dron.id, " ha terminado")

        
        if(not objetivoDetectado):
            print()
            print("----------------------")
            print()
            print("Objetivo NO encontrado")
            print()
            print("----------------------")
        

        print("Check if drones have finished")
        if(allDronesFinished(drones)):
            print("Todos los drones han terminado.")
            print("El resultado de la búsqueda ha sido: ", end="")
            if objetivoDetectado:
                print(" ***** ÉXITO ***** ")
                print("Objetivo se encuentra en la posición: ", posicionObjetivo)
                
            else:
                print(" *** FRACASO ***")
            
            print("**************************************************************************")
            finishMonitor = True
            break
        
        print("**************************************************************************")
        time.sleep(0.2)


def alertCommands():
    global objetivoDetectado
    
    while len(drones) == 0:
        time.sleep(0.5)
        print("Venga tu")
        pass
    print("LEN?????", len(drones))
    d = drones[0]
    while(not (objetivoDetectado or allDronesFinished(drones))):
        time.sleep(0.5)
        if(len(d.vehicle.commands) < 8):
            print("AAAAAAHHHHHH FILHO DE PUTA AGORA SIM ENTENDO, LEN: ", len(d.vehicle.commands))

        if(not d.vehicle.home_location):
            print("NO TIENE CASA")
        #print("ObjetivoDetectado: ", objetivoDetectado, "||| allFInished: ", allDronesFinished(drones))

    print("ALERTA TERMINADO")

if(alert):
    threadAlert = threading.Thread(target=alertCommands)
    thread_list_start.append(threadAlert)

if(output):
    threadMonitor = threading.Thread(target=monitorDrones)
    thread_list_start.append(threadMonitor)

print("Numero de threads: ",len(thread_list_start))

for thread in thread_list_start:
    thread.start()

for thread in thread_list_start:
    thread.join()


print("TUTUTUTUTU")
#for i in range(num_drones):
#    nuevoDron(i)




"""
def runMission(i):

    controladores[i].despega(20)

    print("mode antes: ", controladores[i].vehicle.mode)
    controladores[i].vehicle.mode = dk.VehicleMode("AUTO")
    print("mode despues: ", controladores[i].vehicle.mode)
    controladores[i].createMission(rutasMultDrones[i])
    controladores[i].executeMission()

#Ver una forma de poder crear un archivo de la ruta en lugar de pasar los objetos de las posiciones a recorrer
thread_list_mission = []
for i in range(num_drones):
    thread = threading.Thread(target=runMission, args=(i,))
    thread_list_mission.append(thread)

for thread in thread_list_mission:
    thread.start()

for thread in thread_list_mission:
    thread.join()
"""


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

#controladores[0].recorreArea(file = "puntosPoligono.txt", granularity = 10)

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
