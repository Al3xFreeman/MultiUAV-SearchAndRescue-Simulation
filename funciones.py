from cmath import atan, cos, sin
from tabnanny import check
from turtle import degrees
import dronekit as dk
import dronekit_sitl as dk_sitl
import argparse
import time
import math
import pyproj

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




    def generateRoute(self, file, granularity):
        """
        Genera el recorrido que tiene que hacer para una determinada área
        """

        coordenadas = readFile(file)

        poligono = estructuraPoligono(coordenadas)

        if not checkPoligono(poligono):
            return "ERROR"
        
        matriz = generaMatriz(poligono, granularity)

        puntosDentro = generaPuntos(poligono, matriz)

        ruta = solveTSP(puntosDentro)

        return generateWaypoints(self.vehicle.location.global_frame, ruta)


#Ángulo entre dos puntos y lat y lon dando un punto, la dist y el angulo
#https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
def get_angle_from_point(point1, point2):
    """
    Recibe:
        - Dos puntos (LocationGlobal)
    Devuelve:
        - El ángulo (bearing) entre dos puntos en una superficie esférica (de punto1 a punto2)
        - (DE MOMENTO NO) El ángulo inverso (de punto2 a punto1)
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

    return (fwd_azimuth - 90, distance)


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


def readFile(file):
    """
    Lee un archivo con distintos puntos que ocnforman un polígono
    y devuelve una serie de puntos (NO el objeto de dronekit, solo los puntos parseados, faltaría la altura)
    """

    f = open(file, "r")
    puntos = list(map(lambda line: line.split(', '), f.readlines())) #Separa cada coordenada en lat y lon

    return puntos



def estructuraPoligono(coords):
    """
    Recibe una serie de coordenadas y devuelve una serie de ordenadas y abscisas.
    (0,0) será el primer punto del polígono.
    """

    angle_dist = []
    origin = LocationGlobal(coords[0][0], coords[0][1])
    print(origin)
    #coords.append(coords[0])
    for coord in coords[1:]:
        loc = LocationGlobal(coord[0], coord[1])
        print(loc)
        angle_dist.append(get_angle_from_point(origin, loc))

    for elem in angle_dist:
        print("ANGLE DIST:", elem)

    points = []
    points.append((0,0))
    for elem in angle_dist:
        points.append(get_location_dist_angle(elem[0], elem[1]))

    print(points)
    l = list(zip(*points))

#    fig, ax = plt.subplots()
#    ax.scatter(l[0], l[1])
#
#    for i, txt in enumerate(points):
#        ax.annotate(i, (points[i][0], points[i][1]))
#
#    plt.show()

    return points

def checkPoligono(poligono):
    """
    A partir de un polígono (una serie de puntos),
    comprueba si es válido, es decir, si no se corta a si mismo.
    """
    return True

def generaMatriz(poligono, granularidad = 25):
    """
    A partir de un polígono (una serie de puntos),
    procesa el polígono para saber qué cuadrado inscribe a dicho polígono
    A partir de dicho cuadrado, genera una malla de puntos, separados por la granularidad indicada.
    """
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
    largo = abs(lado_izq - lado_derecho)
    ancho = abs(lado_arriba - lado_abajo)

    punto_x = lado_izq
    punto_y = lado_abajo

    while(punto_x < lado_derecho):
        punto_y = lado_abajo
        while(punto_y < lado_arriba):
            puntos.append((punto_x, punto_y))
            punto_y += granularidad
        
        punto_x += granularidad

    print("Total puntos: ", len(puntos))


def generaPuntos(poligono, matriz):
    """
    A partir de un polígono (una serie de puntos) y una matriz,
    genera una lista de todos los puntos de la matriz que estén dentro dentro de dicho polígono.
    """

def solveTSP(points):
    """
    Recibe una lista de puntos a recorrer y encuentra una solución óptima (o cercana a la óptima)
    para recorrer todos los puntos.
    """

def generateWaypoints(origen, points):
    """
    Recibe un punto de origen (donde esté el dron), que será donde vuelva cuando acabe toda la ruta,
    también recibe una lista de puntos y genera waypoints para poder subir la misión al dron.
    """


