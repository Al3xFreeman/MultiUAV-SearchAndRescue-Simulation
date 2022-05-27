import dronekit as dk
import time
import math
import threading
import json

from typing import List

#Local source files
import funciones as func
#from funciones import *
import generadorRutas as genRut
#from generadorRutas import *
from shapely.geometry import shape
from shapely.geometry.polygon import Polygon, Point

class droneMissionSimualtion():
    def __init__(self, id, file, numDrones = 1, granularity = 40, output = False, alert = False, automateGranularity = False):
        self._id = id
        self._file = file
        self._numDrones = numDrones
        self._granularity = granularity
        self._output = output
        self._alert = alert
        self._automateGranularity = automateGranularity
        #self._homeLat = homeLat
        #self._homeLon = homeLon

        #extract the needed information from the GeoJSON file
        self.polygons : List[Polygon] = []
        self.homes : List[Point] = []

        #TODO Check if geoJSON is valid
        dataFile = json.load(self._file)
        #print("QUE HAY AQUÍ ", dataFile)

        for elem in dataFile["features"]:
            if(elem["geometry"]["type"] == "Polygon"): #it is a polygon
                fileShapely: Polygon = shape(elem["geometry"])
                self.polygons.append(fileShapely)
                print("SHAPELY POL", fileShapely.exterior.coords)                
            elif(elem["geometry"]["type"] == "Point"): #it is a point -> home for UAVs The init point will be the first home
                pointShapely : Point = shape(elem["geometry"])
                self.homes.append(pointShapely)
                print("SHAPELY HOME ", pointShapely)

        if len(self.homes) == 0: #If no home was chosen, it will be the centroid of the polygon
            self.home = self.polygons[0].centroid
            print("HOME IS THE CENTROID")
            self._homeLat = self.home.coords[0][1]
            self._homeLon = self.home.coords[0][0]
        else:
            print("HOME INDICATED IN GEOJSON")
            self._homeLat = self.homes[0].coords[0][1]
            self._homeLon = self.homes[0].coords[0][0]

    def executeMission(self):
        print("Iniciando misión con ID: ", self._id)            
        print("NUMERO DE DRONES: ", self._numDrones)
        time.sleep(2)

        #controladores : List[ControladorDron] = [] 
        sims : List[func.IniciaSITL] = []
        drones : List[func.ControladorDron] = []
        self._d = drones
        modo = genRut.Modos.Single

        objetivoDetectado = False

        #print("VALOR DE OBJETIVO DETECTADO: ", objetivoDetectado)
        posicionObjetivo = None

        #extract the needed information from the GeoJSON file
        #polygons = []
        #homes = []

        #TODO Check if geoJSON is valid
        #dataFile = json.load(self._file)
        #print("QUE HAY AQUÍ ", dataFile)
        """
        for elem in dataFile["features"]:
            if(elem["geometry"]["type"] == "Polygon"): #it is a polygon
                fileShapely: Polygon = shape(elem["geometry"])
                polygons.append[fileShapely]
                print("SHAPELY POL", fileShapely.exterior.coords)                
            elif(elem["geometry"]["type"] == "Point"): #it is a point -> home for UAVs
                pointShapely : Point = shape(elem["geometry"])
                homes.append[pointShapely]
                print("SHAPELY HOME ", pointShapely)
        """
        #coords= dataFile["features"][0]["geometry"]
        #print("POLIGONO", coords)
        #fileShapely: Polygon = shape(coords)
        #print("SHAPELY POL", fileShapely.exterior.coords)

        #For the time being just take the first inputed polygon, but the array will be used in the future to treat multiple polygons
        #Use the feature of shapely of colection of polygons, points, etc...
        generadorRutas = genRut.GeneraRutas(file = "puntosPoligono2.txt", granularity=self._granularity, modo=modo, coordenadas=self.polygons[0], automateGranularity=self._automateGranularity)
        (rutas, coordenadasPol) = generadorRutas.generaRuta()

        if modo == genRut.Modos.Single:
            #Esto sirve para cuando es una sola ruta (Modos.Single) y tenemos multiples drones
            def divideRutaEntreDrones(numDrones, ruta):
                segmentSize = math.floor(len(ruta)/self._numDrones)
                remainingSpots = len(ruta) - (segmentSize * self._numDrones)
                spotsPerDrone = [segmentSize] * self._numDrones
                index = 0
                print("Remaining spots:", remainingSpots)
                while(remainingSpots != 0):
                    
                    if(index % self._numDrones == 0):
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
            
            rutasMultDrones = divideRutaEntreDrones(self._numDrones, rutas[0])
            for i, e in enumerate(rutasMultDrones):
                print("ID: ", i, " | LEN: ", len(e))

        def nuevoDron(id, camaraActivada, ruta):
            time.sleep(id * 0.2)
            print("INICIANDO DRON ID: ", id)
            sim = func.IniciaSITL(self._homeLat, self._homeLon)

            print("SITL INICIADO")

            sims.append(sim)
            #print("CON: ", sim.connection_string)
            
            dron = func.ControladorDron(sim.connection_string, id, len(ruta), lastPoint=ruta[-1], wayPointLocations=ruta)
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

            time.sleep(25)
            print("Cerrando el SITL")
            sim.sitl.stop()


        kafkaClient = func.KafkaClient(hosts="localhost:9092")

        topicCoords = kafkaClient.topics["mapaDronesSetup"]
        producerCoords = topicCoords.get_sync_producer()

        data = {}
        data['type'] = "setup"
        data['homeLat'] = self._homeLat
        data['homeLon'] = self._homeLon
        data['pol'] = coordenadasPol
        data['numDrones'] = self._numDrones

        setupMsg = func.json.dumps(data)
        producerCoords.produce(setupMsg.encode('ascii'))


        thread_list_start = []
        camaraActivada = True

        for i in range(self._numDrones):
            threadDron = threading.Thread(target=nuevoDron, args=(i,camaraActivada, rutasMultDrones[i]))
            camaraActivada = False
            thread_list_start.append(threadDron)

        def allDronesFinished(drones : List[func.ControladorDron]):
            for drone in drones:
                if not drone.finished:
                    return False
            
            return True

        def allDronesFinishedCommunication(drones : List[func.ControladorDron]):
            for drone in drones:
                if not drone.finishCommunication:
                    return False
            
            return True

        def dronesFinished():
            return len(list(filter(lambda x:x.finished, drones)))

        def dronesPorcentaje():
            recorrido = 0
            total = 0
            for d in drones:
                total += d.numberOfPoints
                if(total == 0): #Rework this, this is just a patch to not have div by 0. Store the prev total if the drone is uploading the commands, that is when its length can be 0.
                    total = 1
                recorrido += d.maxWP

            return recorrido/total

        def communicateDrones():
            time.sleep(30)
            finishComunicate = False
            while(not finishComunicate and not allDronesFinishedCommunication(drones)):
                for dron in drones:
                    if(dron.objetivoEncontrado):
                        print("Procediendo a terminar la ejecución del resto de drones")
                        for d in drones:
                            d.finished = True

                        finishComunicate = True
                        break
                time.sleep(1)
            print("FINISH COMUNICATE")

        def monitorDrones():
            global objetivoDetectado
            
            #Damos tiempo a que se inicien los drones
            time.sleep(30)
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

                        #print("Procediendo a terminar la ejecución del resto de drones")
                        #for d in drones:
                        #    d.finished = True


                for dron in drones:
                    #TODO: Hacer que la comprobación de si han terminado no dependa de si hay o no drones
                    #   Ya que si se están iniciando, peta. Dejarlo como una variable del programa principal que sea controlada por la ejecución de cada dron
                    #print("FINISHED: ", dron.finished)
                    if(not dron.finishCommunication):
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

        if(self._alert):
            threadAlert = threading.Thread(target=alertCommands)
            thread_list_start.append(threadAlert)

        if(self._output):
            threadMonitor = threading.Thread(target=monitorDrones)
            thread_list_start.append(threadMonitor)

        threadCommunication = threading.Thread(target=communicateDrones)
        thread_list_start.append(threadCommunication)

        def produceMissionInfo():
                tiempoVueloTotal = 0
                bateriasNecesarias = 0
                for d in drones:
                    tiempoVueloTotal += d.updateTiempoVuelo()
                    #if(d.tiempoDeVuelo > tiempoVueloMax):
                    #    tiempoVueloMax = d.tiempoDeVuelo
                    bateriasNecesarias += d.bateriasUsadas
                #print("Tiempo vuelo total:", tiempoVueloTotal, "| Tiempo medio:", tiempoVueloTotal/len(drones),"| tiempo Max", tiempoVueloMax, "\nBaterias:", bateriasNecesarias, "| Baterias media:", bateriasNecesarias/len(drones))
                dataEndMission = {}
                dataEndMission['type'] = "update"
                dataEndMission['tiempoVueloTotal'] = tiempoVueloTotal
                dataEndMission['tiempoVueloMedio'] = tiempoVueloTotal/len(drones)
                #dataEndMission['tiempoVueloMax'] = tiempoVueloMax
                dataEndMission['bateriasTotal'] = bateriasNecesarias
                dataEndMission['bateriasMedia'] = bateriasNecesarias/len(drones)
                dataEndMission['dronesFinished'] = dronesFinished()
                dataEndMission['porcentaje'] = dronesPorcentaje()


                setupMsg = func.json.dumps(dataEndMission)
                producerCoords.produce(setupMsg.encode('ascii'))


        def sendResumen():
            time.sleep(60)
            print("START RESUMEN | LEN drones: ", len(drones))
            
            while(not allDronesFinishedCommunication(drones)):
                produceMissionInfo()
                time.sleep(0.033)
            print("END RESUMEN")
            produceMissionInfo()
            
        threadUpdate = threading.Thread(target=sendResumen)
        thread_list_start.append(threadUpdate)

        print("Numero de threads: ",len(thread_list_start))

        for thread in thread_list_start:
            thread.start()

        for thread in thread_list_start:
            thread.join()

        tiempoVueloTotal = 0
        tiempoVueloMax = 0
        bateriasNecesarias = 0

        def calcResumen():
            tiempoVueloTotal = 0
            tiempoVueloMax = 0
            bateriasNecesarias = 0

            for d in drones:
                tiempoVueloTotal += d.tiempoDeVuelo
                if(d.tiempoDeVuelo > tiempoVueloMax):
                    tiempoVueloMax = d.tiempoDeVuelo
                bateriasNecesarias += d.bateriasUsadas

            #Hacer una estadística para ver cuánto afecta que la ruta del dron tenga muchos o pocos giros al tiempo de vuelo total

            print("Tiempo de vuelo total: ", tiempoVueloTotal, "||| Tiempo de vuelo medio: ", tiempoVueloTotal/self._numDrones, "||| Tiempo de vuelo max: ", tiempoVueloMax)
            print("Número de baterias usadas: ", bateriasNecesarias, "||| Baterías usadas de media: ", bateriasNecesarias/self._numDrones)

            
            dataEndMission = {}
            dataEndMission['type'] = "end"
            dataEndMission['tiempoVueloTotal'] = tiempoVueloTotal
            dataEndMission['tiempoVueloMedio'] = tiempoVueloTotal/self._numDrones
            dataEndMission['tiempoVueloMax'] = tiempoVueloMax
            dataEndMission['bateriasTotal'] = bateriasNecesarias
            dataEndMission['bateriasMedia'] = bateriasNecesarias/self._numDrones


            setupMsg = func.json.dumps(dataEndMission)
            producerCoords.produce(setupMsg.encode('ascii'))
            

        calcResumen()

        print("Missión terminada")


        producerCoords.stop()
