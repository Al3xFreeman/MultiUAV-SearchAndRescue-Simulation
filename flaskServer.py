from flask import Flask, render_template, Response
from pykafka import KafkaClient, common


def get_kafka_client():
    return KafkaClient(hosts="localhost:9092")

app = Flask(__name__)

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