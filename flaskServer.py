from flask import Flask, render_template, Response, request
from pykafka import KafkaClient, common
import droneMissionSimulation as sim

def get_kafka_client():
    return KafkaClient(hosts="localhost:9092")

app = Flask(__name__)

missionList = []

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

@app.route("/sendMission", methods=['POST'])
def startMission():
    #missionData = request.values
    #print(missionData)

    #Cada misión requiere de un id, comprobar que sea único
    mission = sim.droneMissionSimualtion(id=1, homeLat=40.453010, homeLon=-3.732698)
    mission.executeMission()