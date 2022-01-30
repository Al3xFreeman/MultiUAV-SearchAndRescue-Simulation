from cmath import cos, sin
import dronekit as dk
import argparse
import time
import math
import pyproj
from shapely.geometry import Point, Polygon

from pymavlink import mavutil

import datetime

import numpy as np
import matplotlib.pyplot as plt

import checks
from dronekit import LocationGlobalRelative, LocationGlobal

from python_tsp.distances import great_circle_distance_matrix
from python_tsp.heuristics import solve_tsp_local_search


#Controlador encargado de centralizar las diversas funciones
# del dron.
#
#Cada función deberá realizar las comprobaciones necesarias previa a su ejecución
# Si alguna fallara, no se realizaría la acción
#
#Deberá ser posible, mediante una flag, controlar si estas comprobaciones se comprueban o no


class IniciaSITL:
    def __init__(self) -> None:
        
        

        self.parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
        self.parser.add_argument('--connect',
                            help="Vehicle connection target string. If not specified, SITL automatically started and used.")
        self.parser.add_argument('--home',
                            help = "Establece el punto de inicio del dron")
        args = self.parser.parse_args()

        startLoc = args.home
        self.connection_string = args.connect
       
        """ De momento siempre empieza en simulador y listo
        # Start SITL if no connection string specified
        if not connection_string:
            import dronekit_sitl
            sitl = dronekit_sitl.start_default()
            connection_string = sitl.connection_string()
        """

        import dronekit_sitl as dk_sitl

        self.sitl = dk_sitl.SITL()

        self.sitl.download('copter', '3.3', verbose=True)
        sitl_args = ['-IO', '--model', 'quad', '--home=40.451110, -3.732398,0,180']
        self.sitl.launch(sitl_args, await_ready=True, restart=True)
        
    def getConnectionString(self):
        '''returned string may be used to connect to simulated vehicle'''
        # these are magic numbers; ArduPilot listens on ports starting
        # at 5760+3*(instance-number)
        port = 5760
        port += 3 * self.instance
        return 'tcp:127.0.0.1:' + str(port)      



class ControladorDron:
    def __init__(self, connect):
        print("Iniciando el controlador del dron")
        
        self.connection_string = connect
        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % self.connection_string)
        self.vehicle = dk.connect(self.connection_string, wait_ready=True)

        while not self.vehicle.home_location:
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not self.vehicle.home_location:
                print(" Waiting for home location ...")
            time.sleep(1)

        print("HOME:", self.vehicle.home_location)



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

        print("CONNECTION STRING: ", self.connection_string)
        print("HOME: ", self.vehicle.home_location)

        print("Leyendo archivo")
        coordenadas = readFile(file)

        print("Estrucutrando polígono")
        poligono = estructuraPoligono(coordenadas)

        #Comprueba si el polígono es válido
        #if not checkPoligono(poligono):
        #     return "ERROR"
        
        print("Generando matriz")
        matriz = generaMatriz(poligono, granularity)

        print("Calculando puntos dentro")
        puntosDentro = generaPuntos(poligono, matriz)

        print("************TSP**********")
        (ruta, coste) = otherTSP(puntosDentro)

        print("Genera Waypoints")
        locationsGlobals = generateWaypoints(puntosDentro, ruta)

        self.createMission(locationsGlobals)

        showGraph = False

        if(showGraph):

            coord_x = []
            coord_y = []
            coord_x_pol = []
            coord_y_pol = []

            for elem in puntosDentro:
                coord_x.append(elem.lon)
                coord_y.append(elem.lat)
            
            for elem in poligono:
                coord_x_pol.append(elem.lon)
                coord_y_pol.append(elem.lat)

            l = [coord_x, coord_y]
            l_pol = [coord_x_pol, coord_y_pol]

            #Pone la Home location en el mapa
            #l[0].append(self.vehicle.location.global_frame.lat)
            #l[1].append(self.vehicle.location.global_frame.lon)

            fig, ax = plt.subplots()
            ax.scatter(l[0], l[1])
            ax.scatter(l_pol[0], l_pol[1], color='red')

            conex_x = [l[0][ruta[-2]]]
            conex_y = [l[1][ruta[-2]]]

            for i, txt in enumerate(puntosDentro):
                ax.annotate(i, (l[0][i], l[1][i]))
                conex_x.append(l[0][ruta[i]])
                conex_y.append(l[1][ruta[i]])

            plt.plot(conex_x, conex_y)

            plt.show()

        return locationsGlobals
    
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

    def executeMission(self, points):

        #self.despega(self, altura)


        print("Starting mission")
        # Reset mission set to first (0) waypoint
        self.vehicle.commands.next=0

        # Set mode to AUTO to start mission
        self.vehicle.mode = dk.VehicleMode("AUTO")
        self.vehicle.send_mavlink
        # Monitor mission. 
        # Demonstrates getting and setting the command number 
        # Uses distance_to_current_waypoint(), a convenience function for finding the 
        #   distance to the next waypoint.

        print("Batería: ", self.vehicle.battery)

        while True:
            nextwaypoint = self.vehicle.commands.next
            print('Distance to waypoint (%s): %s' % (nextwaypoint, self.distance_to_current_waypoint()))
            print("Batería: ", self.vehicle.battery)

            self.checkBattery(94, points)

            if nextwaypoint==len(self.vehicle.commands) - 2: #Skip to next waypoint
                print("Skipping to Waypoint", len(self.vehicle.commands)," when reach waypoint ", len(self.vehicle.commands) - 2)
                self.vehicle.commands.next = len(self.vehicle.commands)
            if nextwaypoint==len(self.vehicle.commands): #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
                print("Exit 'standard' mission when start heading to final waypoint (5)")
                break;
            time.sleep(1)

                
        print('Return to launch')
        self.vehicle.mode = dk.VehicleMode("RTL")

    def checkBattery(self, level, points):
        if(self.vehicle.battery.level < level):
            print("Batería restante baja... VOlviendo a casa para cambio de batería")
            self.vehicle.mode = dk.VehicleMode("RTL")
        
            while(get_distance_metres(self.vehicle.location.global_frame, self.vehicle.home_location) > 10):
                print("Not home yet ||||| Distancia: ", get_distance_metres(self.vehicle.home_location, self.vehicle.location.global_frame))
                time.sleep(0.5)
            
            print("YA EN CASA")

            #Guarda el estado de los comandos actual
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            
            siguiente = self.vehicle.commands.next
            
            #for elem in cmds:
                #print(elem)

            print("CONNECTION STRING: ", self.connection_string )

            #Se desconecta para simular el cambio de batería
            self.vehicle.close()

            #Se vuelve a conectar cuando tiene la nueva batería
            print('Connecting to vehicle on: %s' % self.connection_string)
            self.vehicle = dk.connect(self.connection_string, wait_ready=True)
            self.createMission(locations = points)

            self.vehicle.commands.next = siguiente

            
            #Continúa con la misión
            self.vehicle.mode = dk.VehicleMode("AUTO")
            print("Me había quedado yendo hacia el punto: ", siguiente)




    def recorreArea(self, file, granularity = 25):
        points = self.generateRoute(file, granularity)
        self.executeMission(points)
    
    
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



def estructuraPoligono(coords):
    """
    Recibe una serie de coordenadas.

    Si el parámetro glob es False, devuelve las coordenadas del polígono en relación al origen. (0,0) será el primer punto del polígono.
    Si el parámetro golb es True, devuelve las coordenadas del polígono como LocationGlobal.    
    """

    origin = LocationGlobal(float(coords[0][0]), float(coords[0][1]))

    pointsLocationGlobal = [origin]

    for coord in coords[1:]:
        loc = LocationGlobal(float(coord[0]), float(coord[1]))
        pointsLocationGlobal.append(loc)

    return pointsLocationGlobal
"""
    #Print los puntos del polígono
    coord_x = []
    coord_y = []

    for elem in pointsLocationGlobal:
        coord_x.append(elem.lat)
        coord_y.append(elem.lon)
    
    l = [coord_x, coord_y]


    fig, ax = plt.subplots()
    ax.scatter(l[0], l[1], color='red')

    #for i, txt in enumerate(pointsLocationGlobal):
    #    ax.annotate(i, (l[i][0], l[i][1]))

    plt.show()
""" 


def checkPoligono(poligono):
    """
    A partir de un polígono (una serie de puntos),
    comprueba si es válido, es decir, si no se corta a si mismo.
    """
    return True

#Modificado para que devuelva una serie de GlobalLocation
def generaMatriz(poligono, granularidad = 25):
    """
    Recibe un polígono (una serie de puntos), una granularidad y glob (que determina 
    si se devolverán los puntos en relación a un origen o como LocationGLobal)

    Procesa el polígono para saber qué cuadrado inscribe a dicho polígono
    A partir de dicho cuadrado, genera una malla de puntos, separados por la granularidad indicada.
    """

    #Obtiene todas las latitudes y longitudes y selecciona las máximas y mínimas
    lat = []
    lon = []
    for elem in poligono:
        lat.append(elem.lat)
        lon.append(elem.lon)
    
    lado_izq = min(lat)
    lado_derecho = max(lat)
    lado_arriba = max(lon)
    lado_abajo = min(lon)

    esquina_izq_arr = LocationGlobal(lado_izq, lado_arriba)
    esquina_izq_abajo = LocationGlobal(lado_izq, lado_abajo)
    esquina_derecha_arr = LocationGlobal(lado_derecho, lado_arriba)
    esquina_derecha_abajo = LocationGlobal(lado_derecho, lado_abajo)

    #De momento vamos a tratar la granularidad como 3 niveles:
    # Grande, normal y pequeña

    # Voy a hardcodear primero la normal y luego vamos viendo el resto
    # La voy a establecer a 25m

    puntos = []

    dist_x = get_distance_metres(esquina_izq_arr, esquina_derecha_arr)
    dist_y = get_distance_metres(esquina_izq_arr, esquina_izq_abajo)

    x = 0
    y = 0
    
    while x < (dist_x + granularidad):
        y = 0
        while y < (dist_y + granularidad):
            puntos.append(get_location_metres(esquina_izq_abajo, y, x))
            y += granularidad
        x += granularidad

   
    print("Total puntos: ", len(puntos))
        
    return puntos

#IMPORTANTE:
    #https://automating-gis-processes.github.io/CSC18/lessons/L4/point-in-polygon.html
def generaPuntos(poligono, matriz):
    """
    A partir de un polígono (una serie de puntos) y una matriz,
    genera una lista de todos los puntos de la matriz que estén dentro dentro de dicho polígono.
    """

    pol = []
    for elem in poligono:
        pol.append((elem.lat, elem.lon))
    polygonObj = Polygon(pol)
   
    puntosReturn = []

    for punto in matriz:
        p = Point(punto.lat, punto.lon)
        
        if(p.within(polygonObj)):
            puntosReturn.append(punto)
    
    print("Longitud de puntos dentro:", len(puntosReturn))

    return puntosReturn
    

def otherTSP(points):

    init = datetime.datetime.now()

    latLon = extractLatLon(points)

    dist_matrix = great_circle_distance_matrix(latLon)
    #dist_matrix[:, 0] = 0 #No tiene que terminar donde empieza
    #No se aprecia ninguna diferencia, así que bastante mejor que termine donde empieza

    init = datetime.datetime.now()
    ruta, coste = solve_tsp_local_search(dist_matrix)
    end = datetime.datetime.now()
    ruta.append(ruta[0])
    print("Ruta:", ruta)
    print("Coste: ", coste)

    print("Time exec LOCAL SEARCH: ", (end - init).total_seconds())

    return (ruta, coste)

def extractLatLon(points):
    latLon = []
    
    for p in points:
        latLon.append((p.lat, p.lon))

    return np.array(latLon)


def generateWaypoints(points, orderPoints):
    """
    Recibe un punto de origen (donde esté el dron), que será donde vuelva cuando acabe toda la ruta,
    también recibe una lista de puntos y genera las localizaciones globales para posteriormente
    añadirlas a una misión.
    """

    locations = []

    #La última posición de la ruta es el origen.
    #Lo añadiremos el primero, porque el origen del polígono
    # no tiene por qué ser el "home" del dron.
    locations.append(points[orderPoints[-1]]) 
    #Points es el conjunto de puntos que está dentro del polígono
    #Vamos eligiendo dichos puntos en función de lo que determine la ruta

    for i in range(len(points)):
        locations.append(points[orderPoints[i]])

    return locations