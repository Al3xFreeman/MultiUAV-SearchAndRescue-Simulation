from cmath import cos, sin
from concurrent.futures import thread
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
from dronekit import LocationGlobalRelative, LocationGlobal, Command

from python_tsp.distances import great_circle_distance_matrix
from python_tsp.heuristics import solve_tsp_local_search

from typing import List

from vidDetect import *
import threading


#Controlador encargado de centralizar las diversas funciones
# del dron.
#
#Cada función deberá realizar las comprobaciones necesarias previa a su ejecución
# Si alguna fallara, no se realizaría la acción
#
#Deberá ser posible, mediante una flag, controlar si estas comprobaciones se comprueban o no


class IniciaSITL:
    def __init__(self) -> None:
       
        import dronekit_sitl
        self.sitl = dronekit_sitl.start_default(lat = 40.453010, lon = -3.732698)
        self.connection_string = self.sitl.connection_string()
 
        #import dronekit_sitl as dk_sitl

        #self.sitl = dk_sitl.SITL()

        #self.sitl.download('copter', '3.3', verbose=True)
        #sitl_args = ['-IO', '--model', 'quad', '--home=40.453010, -3.732698,0,180']
        #self.sitl.launch(sitl_args, await_ready=True, restart=True)
        
    def getConnectionString(self):
        '''returned string may be used to connect to simulated vehicle'''
        # these are magic numbers; ArduPilot listens on ports starting
        # at 5760+3*(instance-number)
        port = 5760
        port += 10 * self.sitl.instance
        return 'tcp:127.0.0.1:' + str(port)      


class ControladorDron:
    def __init__(self, connect, id):
        #print("ID = ", id)
        
        self.id = id
        self.connection_string = connect
        self.finished = False
        self.continueExecution = True
        self.posicionObjetivo = None
        
        # Connect to the Vehicle
        #print('Connecting to vehicle on: %s' % self.connection_string)
        self.vehicle = dk.connect(self.connection_string, wait_ready=True)

        #self.download_mission()
        #print(" Waiting for home location", end='')
        #while not self.vehicle.home_location:
        #    print('.', end='')
        #    time.sleep(0.5)
        #print(" Waiting for home location ...", end='')
        #while not self.vehicle.home_location:
        #    cmds = self.vehicle.commands
        #    cmds.download()
        #    cmds.wait_ready()
            
        #    if not self.vehicle.home_location:
        #        print(".", end='')
        #    time.sleep(1)
        #print("ale")
            
        print("ID: ", id, " ||| ConnectionString: ", self.connection_string, "||| HOME:", self.vehicle.home_location)
        self.outputMode = "Normal"
        self.bateriasCambiadas = 0 #Veces que ha ido a repostar las baterías
        self.file = None

       
    def iniciaDron(self, camaraActivada):
         #Threads para:
        # - La ejecución del movimiento del dron
        # - El funcionamiento de la cámara
        # - La comprobación de la detección
        threadsDron = []

        if(camaraActivada):
            self.video = VideoDetect()

        threadMission = threading.Thread(target=self.executeMission)

        if(camaraActivada):
            threadVideo = threading.Thread(target=self.detectaVideoThread)
            threadDetection = threading.Thread(target=self.checkDetection)

        if(camaraActivada):
            threadsDron.append(threadVideo)
            threadsDron.append(threadDetection)

        threadsDron.append(threadMission)

        for i, t in enumerate(threadsDron):
            print("THREAD: ", i)
            t.start()
        
        for t in threadsDron:
            t.join()

    
    def detectaVideoThread(self):
        self.video.runVideoDetection()
    
    def checkDetection(self):
        
        while(not self.video.detected_cow):
            #print("No se ha encontrado a la vaca ):")
            time.sleep(2)
            pass

        print("VACA ENCONTRADA!!!")
        time.sleep(0.3)
        self.posicionObjetivo = self.vehicle.location.global_frame
        self.video.ejecuta_video = False
        self.continueExecution = False
        



    def getInfo(self, sep = " ||| "):
        print_id = "ID: " + str(self.id).ljust(3)
        if self.vehicle is not None:
            #print("OUTPURMODE: ", self.outputMode)
            print_mode = "Modo: " + self.vehicle.mode.name
            if(self.vehicle.mode.name == "RTL"):
                
                print(print_id, sep, print_mode)
                
            elif(self.vehicle.mode.name == "AUTO" or self.vehicle.mode.name == "GUIDED"):
                print_wp = "Waypoint: " + str(self.vehicle._current_waypoint).ljust(3) + " de " + str(len(self.vehicle.commands)).ljust(3)
                if self.distance_to_current_waypoint() is not None:
                    print_dist_wp = "Distancia al siguiente punto: " + str(self.distance_to_current_waypoint())
                else:
                    print_dist_wp = ""

                if self.vehicle.battery is not None:
                    print_bat = "Bateria: " + str(self.vehicle.battery.level)
                else:
                    print_bat = ""

                print(print_id, sep, print_mode, sep, print_bat, sep, print_wp, sep, print_dist_wp)
            else:
                print(print_id, sep, print_mode, sep, "Mira no sé qué poner", sep)
        else:
            print(print_id, " VEHICULO NO INICIADO")
    def setWPFile(self, file):
        self.file = file

    #Despega el dron a la altura especificada
    def despega(self, altura):
        """
            Comprobaciones:
                - Altura es válida
                - is_armable
                - Modo
                - Armado
        """
        if(altura < 0):
            print("Error: Altura ha de ser un valor positivo")
            return

        checks.pre_arm(self.vehicle)

        checks.arming(self.vehicle)

        print("Despega")
        self.vehicle.simple_takeoff(altura)

        #Espera a que llegue a la altura especificada
        #Si se mandara otra acci´pn antes de ello, se cancelaría la acción de despegar
        checks.esperaAltura(self.id, self.vehicle, altura)

    #https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
    def mueve(self, **kwargs):
        tiempo = kwargs.get('tiempo', None) #Tiempo a moverse a la velocidad establecida
                                            
        distancia = kwargs.get('dist', None) #Distancia a recorrer

        vel = kwargs.get('vel', None) #Controlling vehicle movement using velocity is much smoother than using position when there are likely to be many updates (for example when tracking moving objects).

        dir = kwargs.get('dir', None) #Yaw Axis -> Vehicle.attitude



    def uploadMissionFromUploadedFile(self):
        if self.file == None:
            return "error, no existe el archivo"
        missionList = self.readmission(self.file)

        print ("\nUpload mission from a file: %s" % self.file)

        #Clear existing mission from vehicle
        print(' Clear mission')
        cmds = self.vehicle.commands
        cmds.clear()

        #Add new mission to vehicle
        for command in missionList:
            cmds.add(command)

        print(' Upload mission')
        self.vehicle.commands.upload()

    def uploadMissionFromFile(self, file):
        if file == None:
            return "error, no existe el archivo"
        missionList = self.readmission(file)

        print ("\nUpload mission from a file: %s" % file)

        #Clear existing mission from vehicle
        print(' Clear mission')
        cmds = self.vehicle.commands
        cmds.clear()

        #Add new mission to vehicle
        for command in missionList:
            cmds.add(command)

        print(' Upload mission')
        self.vehicle.commands.upload()
    def readmission(self, aFileName):
        """
        Load a mission from a file into a list.

        This function is used by upload_mission().
        """
        print("Reading mission from file: %s\n" % aFileName)

        missionlist=[]
        with open(aFileName) as f:
            for i, line in enumerate(f):
                if i==0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:
                    linearray=line.split('\t')
                    ln_index=int(linearray[0])
                    ln_currentwp=int(linearray[1])
                    ln_frame=int(linearray[2])
                    ln_command=int(linearray[3])
                    ln_param1=float(linearray[4])
                    ln_param2=float(linearray[5])
                    ln_param3=float(linearray[6])
                    ln_param4=float(linearray[7])
                    ln_param5=float(linearray[8])
                    ln_param6=float(linearray[9])
                    ln_param7=float(linearray[10])
                    ln_autocontinue=int(linearray[11].strip())
                    cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                    missionlist.append(cmd)
        return missionlist

    def uploadMission(self, waypoints):
        """
        Recibe una serie de waypoints y sube la misión al dron
        """
        cmds = self.vehicle.commands

        print(" Clear any existing commands")
        cmds.clear() 

        print(" Define/add new commands.")

        #Este igual lo quitamos
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

        for wp in waypoints:
            cmds.add(wp)

        #TODO - Ver una forma de hacer esto mejor
        #add dummy waypoint (same as the last one, lets us know when have reached destination)
        cmds.add(cmds[-1])

        cmds.upload()

    def download_mission(self):
        """
        Download the current mission from the vehicle.
        """
        cmds = self.vehicle.commands
        cmds.download()
        print("Descargando cosis")
        cmds.wait_ready() # wait until download is complete.

#Hacer que en cuanto se suba la misión se guarde en un archivo la ruta para poder subirla en cualquier momento
    def createMission(self, locations, altura = 20):
        """
        Crea la misión en base a los puntos generados.
        Se puede especificar la altura a la que se ejecutará la misión con el parámetro "altura".
        """

        self.locations = locations

        cmds = self.vehicle.commands

        print(" Clear any existing commands")
        cmds.clear() 
    
        print(" Define/add new commands.")
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(dk.Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altura))

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
        missionitem=self.vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
        distancetopoint = get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint

    def executeMission(self):

        print("Starting mission")
        # Reset mission set to first (0) waypoint
        self.vehicle.commands.next=0

        # Set mode to AUTO to start mission
        self.outputMode = "Normal"
        self.vehicle.mode = dk.VehicleMode("AUTO")
        #self.vehicle.send_mavlink #Para mandar comandos

        while self.continueExecution:
            nextwaypoint = self.vehicle.commands.next
            #print("ID: ", self.id, "||| POS: ", self.vehicle.location.global_frame, '||| Dist to WP (%s): %s' % (nextwaypoint, self.distance_to_current_waypoint()), end = '')
            if self.vehicle.battery.level == None:
                lvl = 0
            else:
                lvl = self.vehicle.battery.level

            #print("Batería: ", lvl  + (self.bateriasCambiadas * 45), "||||| Batería Real del sim: ", self.vehicle.battery.level)
            
            self.checkBattery(83, lvl)

            if nextwaypoint==len(self.vehicle.commands) - 2: #Skip to next waypoint
                print("Skipping to Waypoint", len(self.vehicle.commands)," when reach waypoint ", len(self.vehicle.commands) - 2)
                self.vehicle.commands.next = len(self.vehicle.commands)
            if nextwaypoint==len(self.vehicle.commands): #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
                print("Exit 'standard' mission when start heading to final waypoint (5)")
                break;
            time.sleep(1)

        if (not self.continueExecution):
            print("La ejecución se paró por haber encontrado al objetivo")
        
        self.finished = True
                
        print('Return to launch')
        self.outputMode = "RTL"
        self.vehicle.mode = dk.VehicleMode("RTL")

    def checkBattery(self, level, batlvl):
        if((batlvl + (self.bateriasCambiadas * 45))  < level):
            print("Batería restante baja... Volviendo a casa para cambio de batería")
            self.outputMode = "RTL"
            self.vehicle.mode = dk.VehicleMode("RTL")

            while not self.vehicle.home_location:
                cmds = self.vehicle.commands
                cmds.download()
                cmds.wait_ready()

            while(get_distance_metres(self.vehicle.location.global_frame, self.vehicle.home_location) > 10):
                print("Not home yet ||||| Distancia: ", get_distance_metres(self.vehicle.home_location, self.vehicle.location.global_frame))
                time.sleep(1)
            
            print("YA EN CASA")

            self.bateriasCambiadas += 1

            #Guarda el estado de los comandos actual
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            
            siguiente = self.vehicle.commands.next
            

            print("CONNECTION STRING: ", self.connection_string )

            #Se desconecta para simular el cambio de batería
            #print("ALTURA???", self.vehicle.location.global_frame.alt)
            while (self.vehicle.location.global_frame.alt > 0.01) and (self.vehicle.location.global_frame.alt > (self.vehicle.home_location.alt + 0.1)):
                #print("ALTURA???", self.vehicle.location.global_frame.alt)
                time.sleep(1)
            self.outputMode = "Normal"
            self.vehicle.mode = dk.VehicleMode("GUIDED")
            self.vehicle.close()


            print("Recargando batería")
            time.sleep(5)
            

            #Se vuelve a conectar cuando tiene la nueva batería
            print("VECES QUE HA IDO A RECARGAR: ", self.bateriasCambiadas)
            print('Connecting to vehicle on: %s' % self.connection_string)
            self.vehicle = dk.connect(self.connection_string, wait_ready=True)
            self.outputMode = "Normal"
            self.vehicle.mode = dk.VehicleMode("GUIDED")

            print("CASA Recarga: ", self.vehicle.home_location)
            print(not self.vehicle.home_location)
            while not self.vehicle.home_location:
                comm = self.vehicle.commands
                comm.download()
                comm.wait_ready()
                if not self.vehicle.home_location:
                    print(" Waiting for home location ...")
                time.sleep(1)

            print("CASA Recarga: ", self.vehicle.home_location)


            self.createMission(self.locations)

            self.vehicle.commands.next = siguiente - 1
            
            #Vuelve a despegar
            self.despega(20)
            #Continúa con la misión
            self.outputMode = "Normal"
            self.vehicle.mode = dk.VehicleMode("AUTO")

            print("Último punto visitado: ", self.vehicle.commands.next)

    
    
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
