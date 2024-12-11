import paho.mqtt.client as mqtt
import json
import cv2
import numpy as np
import sounddevice as sd
import pygame  # Add pygame for joystick handling
import time

class DroneCommunication:
    def __init__(self, topic, gps_topic, lte_topic, audio_topic, video_address="video.mp4", start_stream=True, host="3.110.177.25", port=1883, mavlink_port="127.0.0.1:14550") -> None:
        # Initialize MQTT client
        self.topic = topic
        self.gps_topic = gps_topic
        self.lte_topic = lte_topic
        self.audio_topic = audio_topic
        self.video_address = video_address
        self.start_stream = start_stream
        self.host = host
        self.port = port
        self.mavlink_port = mavlink_port
        
        # Initialize joystick
        pygame.init()
        pygame.joystick.init()
        self.joystick_connected = pygame.joystick.get_count() > 0
        if self.joystick_connected:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Joystick connected: {self.joystick.get_name()}")
        else:
            print("No joystick connected.")
        
        # Create MQTT client
        self.client = mqtt.Client()
        self.client.on_message = self.on_message
        self.client.connect(self.host, self.port)
        
        # Start MQTT loop
        self.client.loop_start()

    # Callback to handle incoming MQTT messages
    def on_message(self, client, userdata, message):
        payload = message.payload
        message_info = f"Received message from {message.topic}: {payload}"

        try:
            if message.topic == self.gps_topic:
                gps_data = json.loads(payload)
                print(f"GPS Data: Latitude={gps_data['latitude']}, Longitude={gps_data['longitude']}")

            elif message.topic == self.lte_topic:
                lte_data = json.loads(payload)
                print(f"LTE Signal Strength: {lte_data['signal_strength']}")

            else:
                print(f"Unrecognized topic {message.topic}: {payload}")

        except Exception as e:
            print(f"Error processing message from topic {message.topic}: {e}")

    # Function to simulate video feed
    def simulate_video_feed(self):
        # Create a simple image for video stream
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, 'Drone Video Feed', (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        _, img_encoded = cv2.imencode('.jpg', img)
        video_bytes = img_encoded.tobytes()
        self.client.publish(self.topic + "/video", video_bytes)
        print(f"Published video data of length {len(video_bytes)} bytes")

    # Function to simulate GPS data
    def simulate_gps_data(self):
        gps_data = {
            "latitude": 37.7749,
            "longitude": -122.4194
        }
        self.client.publish(self.gps_topic, json.dumps(gps_data))
        print(f"Published GPS Data: Latitude={gps_data['latitude']}, Longitude={gps_data['longitude']}")

    # Function to simulate LTE signal data
    def simulate_lte_signal_data(self):
        lte_data = {
            "signal_strength": -75
        }
        self.client.publish(self.lte_topic, json.dumps(lte_data))
        print(f"Published LTE Signal Strength: {lte_data['signal_strength']}")

    # Function to capture and publish audio
    def audio_callback(self, indata, frames, time, status):
        if status:
            print(f"Audio status: {status}")
        audio_bytes = indata.tobytes()
        self.client.publish(self.audio_topic, audio_bytes)

    # Function to handle joystick input and publish commands
    def handle_joystick_input(self):
        pygame.event.pump()  # Update joystick state
        if self.joystick_connected:
            for button in range(self.joystick.get_numbuttons()):
                if self.joystick.get_button(button):  # If button is pressed
                    if button == 0:  # Button 1
                        self.client.publish(self.topic + "/commands", json.dumps({"action": "fly"}))
                        print("Button 1 pressed: Sending fly command")
                    elif button == 1:  # Button 2
                        self.client.publish(self.topic + "/commands", json.dumps({"action": "land"}))
                        print("Button 2 pressed: Sending land command")
                    elif button == 2:  # Button 3
                        self.client.publish(self.topic + "/commands", json.dumps({"action": "hover"}))
                        print("Button 3 pressed: Sending hover command")
                    elif button == 3:  # Button 4
                        self.client.publish(self.topic + "/commands", json.dumps({"action": "stop"}))
                        print("Button 4 pressed: Sending stop command")

    # Main function to start streaming and handle tasks
    def start(self):
        # Start capturing and publishing audio
        sample_rate = 44100  # 44.1 kHz
        channels = 1  # Mono
        with sd.InputStream(samplerate=sample_rate, channels=channels, callback=self.audio_callback):
            try:
                while True:  # Main loop
                    if self.start_stream:
                        self.simulate_video_feed()
                    self.simulate_gps_data()
                    self.simulate_lte_signal_data()
                    self.handle_joystick_input()
                    time.sleep(1)  # Simulate a short delay between data publishing
            except KeyboardInterrupt:
                print("Stopping program.")
                self.client.loop_stop()  # Stop MQTT loop
                self.client.disconnect()  # Disconnect from MQTT broker
                pygame.quit()  # Clean up pygame resources

# Create an instance of the DroneCommunication class
drone_communication = DroneCommunication(
    topic="/drone",
    gps_topic="/drone/gps",
    lte_topic="/drone/lte_signal",
    audio_topic="/drone/audio"
)

# Start the communication
drone_communication.start()
