from cmath import atan, cos, sin
from tabnanny import check
from turtle import degrees, pu
from venv import create
import dronekit as dk
import dronekit_sitl as dk_sitl
import argparse
import time
import math
import pyproj
from shapely.geometry import Point, Polygon

from pymavlink import mavutil

import datetime
from scipy.spatial import distance_matrix


#Para fixear mlrose
import six
import sys
sys.modules['sklearn.externals.six'] = six
import mlrose

from collections import deque

import numpy as np
import matplotlib.pyplot as plt

import checks
from dronekit import LocationGlobalRelative, LocationGlobal

#Controlador encargado de centralizar las diversas funciones
# del dron.
#
#Cada función deberá realizar las comprobaciones necesarias previa a su ejecución
# Si alguna fallara, no se realizaría la acción
#
#Deberá ser posible, mediante una flag, controlar si estas comprobaciones se comprueban o no


class ControladorDron:
    def __init__(self):
        print("Iniciando el controlador del dron")
        
        self.parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
        self.parser.add_argument('--connect',
                            help="Vehicle connection target string. If not specified, SITL automatically started and used.")
        args = self.parser.parse_args()

        connection_string = args.connect
        sitl = None

        # Start SITL if no connection string specified
        if not connection_string:
            import dronekit_sitl
            sitl = dronekit_sitl.start_default()
            connection_string = sitl.connection_string()

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = dk.connect(connection_string, wait_ready=True)



    #Setup del dron
    def inicia():
        print("Iniciado")



    #Despega el dron a la altura especificada
    """
    Comprobaciones:
        - Altura es válida
        - is_armable
        - Modo
        - Armado
    """
    def despega(self, altura):

        if(altura < 0):
            print("Error: Altura ha de ser un valor positivo")
            return

        checks.pre_arm(self.vehicle)

        checks.arming(self.vehicle)

        print("Despega")
        self.vehicle.simple_takeoff(altura)

        #Espera a que llegue a la altura especificada
        #Si se mandara otra acci´pn antes de ello, se cancelaría la acción de despegar
        checks.esperaAltura(self.vehicle, altura)


    #https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
    def mueve(self, **kwargs):
        tiempo = kwargs.get('tiempo', None) #Tiempo a moverse a la velocidad establecida
                                            
        distancia = kwargs.get('dist', None) #Distancia a recorrer

        vel = kwargs.get('vel', None) #Controlling vehicle movement using velocity is much smoother than using position when there are likely to be many updates (for example when tracking moving objects).

        dir = kwargs.get('dir', None) #Yaw Axis -> Vehicle.attitude



    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint. 
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint==0:
            return None
        missionitem=self.vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
        distancetopoint = get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint
        

    def uploadMission(self, waypoints):
        """
        Recibe una serie de waypoints y sube la misión al dron
        """



    def download_mission(self):
        """
        Download the current mission from the vehicle.
        """
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready() # wait until download is complete.




    def generateRoute(self, file, granularity, glob = False):
        """
        Genera el recorrido que tiene que hacer para una determinada área
        """

        coordenadas = readFile(file)

        poligono = estructuraPoligono(coordenadas, glob)

        #Comprueba si el polígono es válido
        #if not checkPoligono(poligono):
        #     return "ERROR"
        
        matriz = generaMatriz(poligono, granularity, glob)

        #print(matriz)

        puntosDentro = generaPuntos(poligono, matriz, glob)

        distanceMatrix = distance_matrix(puntosDentro, puntosDentro)
        
        print(distanceMatrix)

        (ruta, coste) = solveTSP(puntosDentro)

        locationsGlobals = generateWaypoints(self.vehicle.location.global_frame, puntosDentro, ruta)

        self.createMission(locationsGlobals)


        l = list(zip(*puntosDentro))
        fig, ax = plt.subplots()
        ax.scatter(l[0], l[1])

        conex_x = [l[0][ruta[-1]]]
        conex_y = [l[1][ruta[-1]]]

        for i, txt in enumerate(puntosDentro):
            ax.annotate(i, (l[0][i], l[1][i]))
            conex_x.append(l[0][ruta[i]])
            conex_y.append(l[1][ruta[i]])

        plt.plot(conex_x, conex_y)

        plt.show()

    
    def createMission(self, locations):

        cmds = self.vehicle.commands

        print(" Clear any existing commands")
        cmds.clear() 
    
        print(" Define/add new commands.")
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

        for loc in locations:
            cmds.add(dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, loc.lat, loc.lon, 11))
        
        #add dummy waypoint (same as the last one, lets us know when have reached destination)
        cmds.add(dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, locations[-1].lat, locations[-1].lon, 11))

        cmds.upload()

    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint. 
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint==0:
            return None
        missionitem = self.vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
        distancetopoint = get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint

    def executeMission(self):

        #self.despega(self, altura)


        print("Starting mission")
        # Reset mission set to first (0) waypoint
        self.vehicle.commands.next=0

        # Set mode to AUTO to start mission
        self.vehicle.mode = dk.VehicleMode("AUTO")

        # Monitor mission. 
        # Demonstrates getting and setting the command number 
        # Uses distance_to_current_waypoint(), a convenience function for finding the 
        #   distance to the next waypoint.

        while True:
            nextwaypoint = self.vehicle.commands.next
            print('Distance to waypoint (%s): %s' % (nextwaypoint, self.distance_to_current_waypoint()))
        
            if nextwaypoint==len(self.vehicle.commands) - 2: #Skip to next waypoint
                print("Skipping to Waypoint", len(self.vehicle.commands)," when reach waypoint ", len(self.vehicle.commands) - 2)
                self.vehicle.commands.next = len(self.vehicle.commands)
            if nextwaypoint==len(self.vehicle.commands): #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
                print("Exit 'standard' mission when start heading to final waypoint (5)")
                break;
            time.sleep(1)

                
        print('Return to launch')
        self.vehicle.mode = dk.VehicleMode("RTL")

    def recorreArea(self, file, granularity = 25):
        self.generateRoute(file, granularity)
        self.executeMission()
    
    
#Ángulo entre dos puntos y lat y lon dando un punto, la dist y el angulo
#https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
def get_angle_from_point(point1, point2):
    """
    Recibe:
        - Dos puntos (LocationGlobal)
    Devuelve:
        - El ángulo (bearing) entre dos puntos en una superficie esférica (de punto1 a punto2)
        - El ángulo inverso (de punto2 a punto1)
        - La distancia entre ambos puntos
    """
    

    #PROBAR ESTO TMB A VER SI DA LO MISMO:
    #https://math.stackexchange.com/questions/330843/angle-between-two-coordinateslatitude-longitude-from-a-position-on-earth#:~:text=If%20a%20spherical%20earth%20is,in%20space%20between%20the%20vectors.

    #https://stackoverflow.com/questions/54873868/python-calculate-bearing-between-two-lat-long

    geodesic = pyproj.Geod(ellps='WGS84')
    fwd_azimuth,back_azimuth,distance = geodesic.inv(point1.lon, point1.lat, point2.lon, point2.lat)

    #print("RESULT: ", fwd_azimuth)
    #print("REVERSE: ", back_azimuth)
    #print("DISTANCIA: ", distance)

    return (fwd_azimuth - 90, distance, back_azimuth)


#Empezando en (0,0) obtener el punto final con un angulo y la dist
#https://www.wyzant.com/resources/answers/601887/calculate-point-given-x-y-angle-and-distance#:~:text=If%20your%20starting%20point%20is,cos%CE%B8%20and%20y%20%3D%20r%20sin%CE%B8.
def get_location_dist_angle(angle, dist):
    """
    Devuelve el punto que se encuentra a una distancia 'dist' con un determinado ángulo 'angle'
    El origen lo trata como si fuera (0,0), ya que este será el primer punto del polígono
    """
    x = dist * cos(math.radians(angle))
    y = dist * sin(math.radians(angle)) * -1

    return (x.real, y.real)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return dk.LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

#TODO Checkear distintos formatos de coordenadas
def readFile(file):
    """
    Lee un archivo con distintos puntos que ocnforman un polígono
    y devuelve una serie de puntos (NO el objeto de dronekit, solo los puntos parseados, faltaría la altura)
    """

    f = open(file, "r")
    puntos = list(map(lambda line: line.split(', '), f.readlines())) #Separa cada coordenada en lat y lon

    return puntos



def estructuraPoligono(coords, glob = False):
    """
    Recibe una serie de coordenadas.

    Si el parámetro glob es False, devuelve las coordenadas del polígono en relación al origen. (0,0) será el primer punto del polígono.
    Si el parámetro golb es True, devuelve las coordenadas del polígono como LocationGlobal.    
    """

    origin = LocationGlobal(coords[0][0], coords[0][1])

    pointsLocationGlobal = [origin]
    angle_dist = []

    for coord in coords[1:]:
        loc = LocationGlobal(coord[0], coord[1])
        pointsLocationGlobal.append(loc)

        angle_dist.append(get_angle_from_point(origin, loc))


    points = []
    points.append((0,0)) #El origen
    for elem in angle_dist:
        points.append(get_location_dist_angle(elem[0], elem[1]))


    if glob:
        return pointsLocationGlobal
    else:
        return points

    """
    Print los puntos del polígono

    l = list(zip(*points))
    fig, ax = plt.subplots()
    ax.scatter(l[0], l[1])

    for i, txt in enumerate(points):
        ax.annotate(i, (points[i][0], points[i][1]))

    plt.show()
    """



def checkPoligono(poligono):
    """
    A partir de un polígono (una serie de puntos),
    comprueba si es válido, es decir, si no se corta a si mismo.
    """
    return True

#Modificado para que devuelva una serie de GlobalLocation
def generaMatriz(poligono, granularidad = 25, glob = False):
    """
    Recibe un polígono (una serie de puntos), una granularidad y glob (que determina 
    si se devolverán los puntos en relación a un origen o como LocationGLobal)

    Procesa el polígono para saber qué cuadrado inscribe a dicho polígono
    A partir de dicho cuadrado, genera una malla de puntos, separados por la granularidad indicada.
    """

    if glob:
        #Obtiene todas las latitudes y longitudes y selecciona las máximas y mínimas
        lat = []
        lon = []
        for elem in poligono:
            lat.append(elem.lat)
            lon.append(elem.lat)
        
        lado_izq = min(lat)
        lado_derecho = max(lat)
        lado_arriba = max(lon)
        lado_abajo = min(lon)

        esquina_izq_arr = LocationGlobal(lado_izq, lado_arriba)
        esquina_izq_abajo = LocationGlobal(lado_izq, lado_abajo)
        esquina_derecha_arr = LocationGlobal(lado_derecho, lado_arriba)
        esquina_derecha_abajo = LocationGlobal(lado_derecho, lado_abajo)

    else:
        l = list(zip(*poligono))

        lado_izq = min(l[0])
        lado_derecho = max(l[0])
        lado_abajo = min(l[1])
        lado_arriba = max(l[1])


    #De momento vamos a tratar la granularidad como 3 niveles:
    # Grande, normal y pequeña

    # Voy a hardcodear primero la normal y luego vamos viendo el resto
    # La voy a establecer a 25m

    puntos = []

    #Devuelve posiciones globales
    if glob:
        dist_x = get_distance_metres(esquina_izq_arr, esquina_derecha_arr)
        dist_y = get_distance_metres(esquina_izq_arr, esquina_izq_abajo)

        x = 0
        y = 0

        while x < dist_x + granularidad:
            y = 0
            while y < dist_y + granularidad:
                puntos.append(get_location_metres(esquina_izq_abajo, y, x))
                y += granularidad

    #Devuelve posiciones relativas a la posición origen
    else:
        punto_x = lado_izq
        punto_y = lado_abajo

        while(punto_x < lado_derecho + granularidad):
            punto_y = lado_abajo
            while(punto_y < lado_arriba + granularidad):
                puntos.append((punto_x, punto_y))
                punto_y += granularidad
            
            punto_x += granularidad


    print("Total puntos: ", len(puntos))
        
    return puntos

#IMPORTANTE:
    #https://automating-gis-processes.github.io/CSC18/lessons/L4/point-in-polygon.html
def generaPuntos(poligono, matriz, glob):
    """
    A partir de un polígono (una serie de puntos) y una matriz,
    genera una lista de todos los puntos de la matriz que estén dentro dentro de dicho polígono.
    """
    polygonObj = Polygon(poligono)

    puntosDentro = []
    puntosReturn = []


    for punto in matriz:
        if glob:
            p = Point(punto.lat, punto.lon)
        else:
            p = Point([punto[0], punto[1]])
        if(p.within(polygonObj)):
            puntosDentro.append(p)
            puntosReturn.append(punto)
    
    print("Longitud de puntos dentro:", len(puntosDentro))
    
    return puntosReturn

    """
    #Imprime los puntos que estén dentro del polígono
    x = []
    y = []

    for i, elem in enumerate(puntosDentro):
        print("punto número ", i, "-> X:", elem.x, ", Y: ", elem.y)
        x.append(elem.x)
        y.append(elem.y)

    fig, ax = plt.subplots()
    ax.scatter(x, y)

    plt.show()
    """
    

#Si lanza un Value error, atraparlo y reducir el número de puntos a la mitad
# Así hasta que no lo de más.
# Los puntos los reduzco de forma aleatoria, hay un 50% de posibilidades de que
#  un punto desaparezca o no y tirando.
def solveTSP(points):
    """
    Recibe una lista de puntos a recorrer y encuentra una solución óptima (o cercana a la óptima)
    para recorrer todos los puntos.
    """

    init = datetime.datetime.now()

    tspProblem = mlrose.TSPOpt(length = len(points), coords= points, maximize = False)
    best_state, best_fitness = mlrose.genetic_alg(tspProblem, mutation_prob = 0.5,
                                              max_attempts = 1000)


    end = datetime.datetime.now()

    print("Time exec: ", (end - init).total_seconds())
    
    print("GENETIC. The best state found is: ", best_state)
    print("GENETIC. The fitness at the best state is: ", best_fitness)

    print("CHECK: ", len(best_state), len(points))

    """

    best_state2, best_fitness2 = mlrose.hill_climb(tspProblem)

    print("HILL CLIMB. The best state found is: ", best_state2)
    print("HILL CLIMB. The fitness at the best state is: ", best_fitness2)

    best_state3, best_fitness3 = mlrose.random_hill_climb(tspProblem)

    print("RANDOM HILL CLIMB. The best state found is: ", best_state3)
    print("RAMDOM HILL CLIMB. The fitness at the best state is: ", best_fitness3)

    best_state4, best_fitness4 = mlrose.simulated_annealing(tspProblem)

    print("SIMULATED ANNEALING. The best state found is: ", best_state4)
    print("SIMULATED ANNEALING. The fitness at the best state is: ", best_fitness4)

    best_state5, best_fitness5 = mlrose.mimic(tspProblem)

    print("MIMIC. The best state found is: ", best_state5)
    print("MIMIC. The fitness at the best state is: ", best_fitness5)
    """

    return (best_state, best_fitness)

def generateWaypoints(origen, points, orderPoints):
    """
    Recibe un punto de origen (donde esté el dron), que será donde vuelva cuando acabe toda la ruta,
    también recibe una lista de puntos y genera las localizaciones globales para posteriormente
    añadirlas a una misión.
    """

    
    
    startIndex = list(orderPoints).index(0)
    print("StartIndex: ", startIndex)
    orderDeque = deque(orderPoints)

    #Rota la lista de puntos a recorrer para dejar al origen en última posición
    orderDeque.rotate(-startIndex - 1)
    order = list(orderDeque)
    print(order)

    locations = []

    #Por cada punto obtiene su coordenada global y crea el waypoint para añadirlo a la misión
    for i in range(len(points)):
        p = points[order[i]]
        locations.append(get_location_metres(origen, p[1], p[0])) #primero va la coordenada y y luego la x
    
    return locations
        
    






def generateRoute(pos, file, granularity):
    """
    Genera el recorrido que tiene que hacer para una determinada área
    """

    coordenadas = readFile(file)

    poligono = estructuraPoligono(coordenadas)

    #if not checkPoligono(poligono):
    #    return "ERROR"
    
    matriz = generaMatriz(poligono, granularity)

    puntosDentro = generaPuntos(poligono, matriz)

    (ruta, coste) = solveTSP(puntosDentro)

    return generateWaypoints(pos, puntosDentro, ruta)



#def distance_matrix(puntos):
