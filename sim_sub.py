import paho.mqtt.client as mqtt
import json

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("/drone/lte_signal")

def on_message(client, userdata, message):
    print(f"Received message: {message.payload.decode()}")
    lte_data = json.loads(message.payload.decode())
    print(f"Signal Strength: {lte_data['signal_strength']}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("3.110.177.25", 1883, 60)

client.loop_forever()
