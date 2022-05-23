from fileinput import filename
from flask import Flask, render_template, Response, request
from pykafka import KafkaClient, common
import droneMissionSimulation as sim
import time
from typing import List

import threading
import json

import generadorRutas as genRut

from shapely.geometry import shape
from shapely.geometry.polygon import Polygon

def get_kafka_client():
    return KafkaClient(hosts="localhost:9092")

app = Flask(__name__)

missionList : List[sim.droneMissionSimualtion] = []
results = []

@app.route("/map")
def index():
    return render_template('index.html')


@app.route("/topic/<topicname>")
def get_messages(topicname):
    client = get_kafka_client()
    def events():
        consumer = client.topics[topicname].get_simple_consumer(
                consumer_group="test3",
                auto_offset_reset=common.OffsetType.LATEST)
        for i in consumer:
            consumer.commit_offsets()
            yield 'data:{0}\n\n'.format(i.value.decode())

        
    return Response(events(), mimetype="text/event-stream")

ALLOWED_EXTENSIONS = {'geojson'}
def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

@app.route("/sendMission", methods=['POST'])
def startMission():
    #missionData = request.values
    #print(missionData)
    print("Nueva misión empezando")
    #print(request.form)
    
    numDrones = int(request.form['numDrones'])
    print("Number of drones: ", numDrones)

    granularity = int(request.form['granularidad'])
    print("Granularity: ", granularity)
    print(request.files)
    if 'fileGeoJSON' not in request.files:
        return ('No está el archivo con el polígono', 401)

    geoJSONfile = request.files['fileGeoJSON']
    print("ARCHIVO", geoJSONfile.filename)

    print("Esperando")
    time.sleep(5)
    print("Ta luego")
    #Cada misión requiere de un id, comprobar que sea único
    mission = sim.droneMissionSimualtion(id=1, file=geoJSONfile, numDrones=numDrones, granularity=granularity, homeLat=40.453010, homeLon=-3.732698, sendKafka=True)
    missionList.append(mission)
    mission.executeMission()
    #return render_template('index.html')
    return ('', 204)

@app.route("/optimizeMission", methods=['POST'])
def optimizeMission():

    numDrones = int(request.form['numDrones'])
    print("Number of drones: ", numDrones)
    granularity = int(request.form['granularidad'])
    print("Granularity: ", granularity)
    print(request.files)
    if 'fileGeoJSON' not in request.files:
        return ('No está el archivo con el polígono', 401)

    geoJSONfile = request.files['fileGeoJSON']
    print("ARCHIVO", geoJSONfile.filename)

    print("INICIANDO MISION DE OPTIMIZACIÓN")

    #TODO Check if geoJSON is valid
    dataFile = json.load(geoJSONfile)
    #print("QUE HAY AQUÍ ", dataFile)
    coords= dataFile["features"][0]["geometry"]
    #print("POLIGONO", coords)
    fileShapely: Polygon = shape(coords)
    print("SHAPELY POL", fileShapely.exterior.coords)

    generadorRutas = genRut.GeneraRutas(file = "puntosPoligono2.txt", granularity=granularity, coordenadas=fileShapely)
    (rutas, coordenadasPol) = generadorRutas.generaRuta()

    
    t_List = []
    
    for i in range(numDrones):

        print("SIM CON ", i + 1, " DRONES")
        threadDron = threading.Thread(target=droneMission, args=(i+1,geoJSONfile, i+1, granularity, rutas, coordenadasPol, True))
        t_List.append(threadDron)
        #Cada misión requiere de un id, comprobar que sea único
        #mission = sim.droneMissionSimualtion(id=i + 1, file=geoJSONfile, numDrones=i + 1, granularity=granularity, homeLat=40.453010, homeLon=-3.732698, sendKafka=False, rutas = rutas, coordenadasPol=coordenadasPol, rutaGenerada=True)
        #missionList.append(mission)
        #results.append(mission.executeMission())
        #return render_template('index.html')

    for t in t_List:
        t.start()
    
    for t in t_List:
        t.join()

    print(results)

    return ('', 204)

def droneMission(id, file, numDrones, granularity, rutas, coordenadasPol, rutaGenerada, homeLat=40.453010, homeLon=-3.732698, sendKafka=False):
    mission = sim.droneMissionSimualtion(id=id, file=file, numDrones=numDrones, granularity=granularity, homeLat=40.453010, homeLon=-3.732698, sendKafka=False, rutas = rutas, coordenadasPol=coordenadasPol, rutaGenerada=rutaGenerada)
    missionList.append(mission)
    results.append(mission.executeMission())
    #return render_template('index.html')


@app.route("/<missionId>/<dronId>/camera")
def cameraDrone(missionId, dronId):
    return Response(missionList[missionId]._d[dronId].video.frameAnnotated,
                    mimetype='multipart/x-mixed-replace; boundary=frame')