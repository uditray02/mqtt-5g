import paho.mqtt.client as mqtt
import json
import time

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.publish("/drone/lte_signal", json.dumps({"signal_strength": "good"}))

client = mqtt.Client()
client.on_connect = on_connect
client.connect("3.110.177.25", 1883, 60)

client.loop_start()

# Keep the loop running to maintain connection and keep publishing
time.sleep(2)  # Sleep for 2 seconds for the message to be sent
client.loop_stop()
client.disconnect()
