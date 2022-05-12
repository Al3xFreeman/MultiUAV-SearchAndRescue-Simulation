from dronekit import LocationGlobalRelative, LocationGlobal
from shapely.geometry import Point, Polygon

from tokenize import Single
import numpy as np
import matplotlib.pyplot as plt

import funciones as func

from typing import List
from enum import Enum

from python_tsp.distances import great_circle_distance_matrix
from python_tsp.heuristics import solve_tsp_local_search

import datetime

class Modos(Enum):
    Single = 1
    mTSP = 2
    Sectores = 3


class GeneraRutas:
    def __init__(self, file, granularity, modo, coordenadas: Polygon):
        #Genera la ruta para recorrer el área.
        #Posteriormente el dron se dirigirá hasta el punto de inicio y empezará a recorrerla
        #Cuando necesite recargar baterías, volverá a su "home".
        #Y cuando termine de recorrer la zona, también volverá a "home"

        self.file = file
        self.granularity = granularity
        self.modo : Modos = modo
        self._coords = coordenadas
        #TODO USe Shapely methods to improve routes generation

    def generaRuta(self) -> List[int]:
        print("***********************************")
        print("Iniciando la generación de rutas...")
        print("***********************************")

        print("Leyendo archivo")
        self.coordenadas = readFile(self.file)
        
        #Temporal file to accept the new coordinates, will change to use shapely functions all the way
        self.shapelyToCoords = shapToCoords(self._coords)

        print("Estrucutrando polígono")
        self.poligono = estructuraPoligono(self.shapelyToCoords)

        #Comprueba si el polígono es válido
        #if not checkPoligono(poligono):
        #     return "ERROR"
        
        print("Generando matriz")
        self.matriz = generaMatriz(self.poligono, self.granularity)

        print("Calculando puntos dentro")
        self.puntosDentro = generaPuntos(self.poligono, self.matriz)


        if self.modo == Modos.Single:
            return (self.genRutaSingle(), self.shapelyToCoords)
        elif self.modo == Modos.mTSP:
            return (self.genRutaMTSP(), self.shapelyToCoords)
        elif self.modo == Modos.Sectores:
            return (self.genRutaSectores(), self.shapelyToCoords)

        #Ver cómo lanzar excepciones
        return []

    def genRutaSingle(self):
       
        print("************TSP**********")
        (self.ruta, coste) = singleTSP(self.puntosDentro)

        print("Genera Waypoints")
        self.locationsGlobals = generateWaypoints(self.puntosDentro, self.ruta)

        return [self.locationsGlobals]

    def genRutaMTSP(self):
        print("Genera ruta mTSP")

    def genRutaSectores(self):
        print("Genera ruta por sectores")

    def printRuta(self):
        coord_x = []
        coord_y = []
        coord_x_pol = []
        coord_y_pol = []

        for elem in self.puntosDentro:
            coord_x.append(elem.lon)
            coord_y.append(elem.lat)
        
        for elem in self.poligono:
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

        conex_x = [l[0][self.ruta[-2]]]
        conex_y = [l[1][self.ruta[-2]]]

        for i, txt in enumerate(self.puntosDentro):
            ax.annotate(i, (l[0][i], l[1][i]))
            conex_x.append(l[0][self.ruta[i]])
            conex_y.append(l[1][self.ruta[i]])

        plt.plot(conex_x, conex_y)

        plt.show()

#TODO Checkear distintos formatos de coordenadas
def readFile(file):
    """
    Lee un archivo con distintos puntos que ocnforman un polígono
    y devuelve una serie de puntos (NO el objeto de dronekit, solo los puntos parseados, faltaría la altura)
    """

    #f = open(file, "r")
    with open(file) as f:
        puntosStr = list(map(lambda line: line.split(', '), f.read().splitlines())) #Separa cada coordenada en lat y lon

    puntos = []
    #Mejorar esto y hacerlo en una sola linea
    for p in puntosStr:
        puntos.append([float(p[0]), float(p[1])])

    return puntos

def shapToCoords(shapelyPolygon: Polygon):
    puntos = []
    
    for p in shapelyPolygon.exterior.coords:
        puntos.append((p[1], p[0]))

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

def checkPoligono(poligono):
    """
    A partir de un polígono (una serie de puntos),
    comprueba si es válido, es decir, si no se corta a si mismo.
    """
    return True

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
    
    lado_abajo = min(lat)
    lado_arriba = max(lat)
    lado_derecho = max(lon)
    lado_izq = min(lon)

    esquina_izq_arr = LocationGlobal(lado_arriba, lado_izq)
    esquina_izq_abajo = LocationGlobal(lado_abajo, lado_izq)
    esquina_derecha_arr = LocationGlobal(lado_arriba, lado_derecho)
    esquina_derecha_abajo = LocationGlobal(lado_abajo, lado_derecho)

    #De momento vamos a tratar la granularidad como 3 niveles:
    # Grande, normal y pequeña

    # Voy a hardcodear primero la normal y luego vamos viendo el resto
    # La voy a establecer a 25m

    puntos = []

    dist_x = func.get_distance_metres(esquina_izq_arr, esquina_derecha_arr)
    dist_y = func.get_distance_metres(esquina_izq_arr, esquina_izq_abajo)

    x = 0
    y = 0
    
    while x < (dist_x + granularidad):
        y = 0
        while y < (dist_y + granularidad):
            puntos.append(func.get_location_metres(esquina_izq_abajo, y, x))
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

def singleTSP(points):

    init = datetime.datetime.now()

    latLon = extractLatLon(points)

    dist_matrix = great_circle_distance_matrix(latLon)
    #dist_matrix[:, 0] = 0 #No tiene que terminar donde empieza
    #No se aprecia ninguna diferencia, así que bastante mejor que termine donde empieza

    init = datetime.datetime.now()
    ruta, coste = solve_tsp_local_search(dist_matrix)
    end = datetime.datetime.now()
    #ruta.append(ruta[0])
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
    #locations.append(points[orderPoints[-1]]) 
    #Points es el conjunto de puntos que está dentro del polígono
    #Vamos eligiendo dichos puntos en función de lo que determine la ruta

    for i in range(len(points)):
        locations.append(points[orderPoints[i]])

    return locations