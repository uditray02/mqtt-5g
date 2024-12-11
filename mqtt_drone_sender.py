import paho.mqtt.client as mqtt
import cv2
import threading
import queue
import time
import json
from pymavlink import mavutil
import logging
import pyaudio


class StreamPublisher:
    def __init__(self, topic, gps_topic, lte_topic, audio_topic, video_address="video.mp4", start_stream=True, host="3.110.177.25", port=1883, mavlink_port="127.0.0.1:14550") -> None:
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.max_inflight_messages_set(20)
        self.client.max_message_size = 10485760
        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish
        self.client.connect(host, port, keepalive=60)
        self.client.loop_start()

        # Subscribe to control topic for camera commands
        self.client.subscribe("/drone/camera/control")
        self.client.on_message = self.on_message  # Set the callback for incoming messages

        self.topic = topic
        self.gps_topic = gps_topic
        self.lte_topic = lte_topic
        self.video_source = video_address
        self.audio_topic = audio_topic
        self.audio_stream = None

        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.sample_rate = 16000
        self.chunk_size = 1024

        self.cam = cv2.VideoCapture(self.video_source)
        if not self.cam.isOpened():
            print("Error: Could not open video source")
            self.frame_rate = 30  # Default frame rate in case of error
        else:
            self.frame_rate = self.cam.get(cv2.CAP_PROP_FPS)
        
        if self.frame_rate == 0:
            print("Warning: FPS is zero. Defaulting to 30 FPS.")
            self.frame_rate = 30  # Default frame rate if FPS is zero
        
        self.frame_time = 1.0 / self.frame_rate
        self.frame_queue = queue.Queue(maxsize=2)

        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.publish_thread = threading.Thread(target=self.publish_frames)
        self.gps_thread = threading.Thread(target=self.publish_gps_data)
        self.battery_thread = threading.Thread(target=self.publish_battery_level)
        self.velocity_thread = threading.Thread(target=self.publish_velocity)
        self.flight_time_thread = threading.Thread(target=self.publish_flight_time)
        self.lte_thread = threading.Thread(target=self.publish_lte_signal)
        self.audio_thread = threading.Thread(target=self.publish_audio)

        self.publish_success_count = 0
        self.publish_total_count = 0

        # Initialize MAVLink connection
        self.master = mavutil.mavlink_connection(mavlink_port, baud=57600)

        # Flight start time (for calculating flight time)
        self.flight_start_time = None

        if start_stream:
            self.capture_thread.start()
            self.publish_thread.start()
            self.gps_thread.start()
            self.battery_thread.start()
            self.velocity_thread.start()
            self.flight_time_thread.start()
            self.lte_thread.start()
            self.audio_thread.start()

    def on_connect(self, client, userdata, flags, rc, *args):
        if rc == 0:
            print("Connected successfully to the MQTT broker.")
        else:
            print(f"Connection failed with result code {rc}")
        print(f"Connected with result code {rc}")
        # Set the flight start time once connected
        self.flight_start_time = time.time()

    def on_publish(self, client, userdata, mid):
        self.publish_success_count += 1

    def on_message(self, client, userdata, message):
        try:
            payload = json.loads(message.payload)
            action = payload.get("action")
            value = payload.get("value")

            if action == "cam_pitch":
                print(f"Received cam_pitch command: {value}")
                # Handle cam_pitch here
            elif action == "cam_yaw":
                print(f"Received cam_yaw command: {value}")
                # Handle cam_yaw here
            elif action == "fly":
                print("Received fly command")
                # Handle the fly action here
            elif action == "land":
                print("Received land command:")
                # Handle the land action here
            elif action == "hover":
                print("Received hover command: ")
                # Handle the hover action here
            elif action == "stop":
                print("Received stop command:")
                # Handle the stop action here
            elif action == "drone_pitch":
                print("Receiving YAW: {value}")
            elif action == "drone_yaw":
                print("RECEIVING PITCH: {value}")
            


        except Exception as e:
            print(f"Error handling message: {e}")
    

    def capture_frames(self):
        print("Capturing from video source: {}".format(self.video_source))
        prev_capture_time = time.time()

        while True:
            current_time = time.time()
            elapsed_time = current_time - prev_capture_time

            if elapsed_time >= self.frame_time:
                ret, img = self.cam.read()
                if not ret:
                    print("Failed to read frame")
                    break

                img_resized = cv2.resize(img, (640, 480))
                if not self.frame_queue.full():
                    self.frame_queue.put(img_resized)
                else:
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put(img_resized)
                    except queue.Empty:
                        pass

                prev_capture_time = current_time
        self.cam.release()

    def publish_frames(self):
        print("Publishing to topic: {}".format(self.topic))
        while True:
            if not self.frame_queue.empty():
                img = self.frame_queue.get()
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
                result, img_encoded = cv2.imencode('.jpg', img, encode_param)
                img_str = img_encoded.tobytes()
                try:
                    result = self.client.publish(self.topic, img_str)
                    if result.rc != mqtt.MQTT_ERR_SUCCESS:
                        print(f"Publish failed with code: {result.rc}")
                    self.publish_total_count += 1
                except Exception as e:
                    print(f"Failed to publish: {e}")

                if self.publish_total_count >= 10:
                    success_rate = self.publish_success_count / self.publish_total_count
                    if success_rate < 0.8:
                        self.frame_rate = max(5, self.frame_rate - 1)
                    elif success_rate > 0.9:
                        self.frame_rate = min(30, self.frame_rate + 1)
                    self.frame_time = 1.0 / self.frame_rate
                    print(f"Adjusted frame rate to: {self.frame_rate}")
                    self.publish_success_count = 0
                    self.publish_total_count = 0
            time.sleep(0.1)

    def publish_gps_data(self):
        print("Publishing GPS data to topic: {}".format(self.gps_topic))
        while True:
            # Mock GPS data, replace with actual GPS data collection logic
            gps_data = {
                'lat': 37.7749,  # Replace with actual latitude
                'lon': -122.4194,  # Replace with actual longitude
                'alt': 100  # Replace with actual altitude
            }
            gps_message = json.dumps(gps_data)
            try:
                result = self.client.publish(self.gps_topic, gps_message)
                if result.rc != mqtt.MQTT_ERR_SUCCESS:
                    print(f"GPS publish failed with code: {result.rc}")
            except Exception as e:
                print(f"Failed to publish GPS data: {e}")
            
            time.sleep(2)  # Publish GPS data every 2 seconds

    def publish_battery_level(self):
        print("Publishing battery level to topic: /drone/battery")
        while True:
            # Request battery status from Pixhawk
            self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component,
                                                     mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

            # Read the battery status message
            msg = self.master.recv_match(type='BATTERY_STATUS', blocking=True)

            if msg:
                # Extract battery voltage and remaining capacity
                battery_voltage = msg.voltages[0] / 1000.0  # Convert to Volts
                battery_level = (msg.current_battery / 1000.0) * 100  # Percentage
                battery_data = {
                    'voltage': battery_voltage,
                    'level': battery_level
                }
                battery_message = json.dumps(battery_data)
                try:
                    result = self.client.publish("/drone/battery", battery_message)
                    if result.rc != mqtt.MQTT_ERR_SUCCESS:
                        print(f"Battery publish failed with code: {result.rc}")
                except Exception as e:
                    print(f"Failed to publish battery data: {e}")

            time.sleep(1)  # Publish battery data every 1 second

    def publish_velocity(self):
        print("Publishing velocity to topic: /drone/velocity")
        while True:
            # Request the LOCAL_POSITION_NED message
            self.master.mav.request_data_stream_send(
                self.master.target_system, 
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 
                1, 1
            )

            # Read the velocity data message
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)

            if msg is None:
                print ("No LOCAL_POSITION_NED message received.")
                continue
            try:
                velocity_data = {
                    'north': msg.vx,
                    'east': msg.vy,
                    'down': msg.vz,
                }
                velocity_message = json.dumps(velocity_data)
                result = self.client.publish("/drone/velocity", velocity_message)
                if result.rc != mqtt.MQTT_ERR_SUCCESS:
                    print(f"Velocity publish failed with code: {result.rc}")
            except Exception as e:
                print(f"Error publishing velocity: {e}")

            time.sleep(1)

    def publish_flight_time(self):
        print("Publishing flight time to topic: /drone/flight_time")
        while True:
            if self.flight_start_time:
                flight_time = time.time() - self.flight_start_time
                flight_time_data = {'flight_time': flight_time}
                try:
                    result = self.client.publish("/drone/flight_time", json.dumps(flight_time_data))
                    if result.rc != mqtt.MQTT_ERR_SUCCESS:
                        print(f"Flight time publish failed with code: {result.rc}")
                except Exception as e:
                    print(f"Error publishing flight time: {e}")

            time.sleep(1)

    def publish_lte_signal(self):
        print("Publishing LTE signal to topic: {}".format(self.lte_topic))
        while True:
            if not self.client.is_connected():
                print("MQTT client not connected. Reconnecting...")
                self.client.reconnect()
            print(f"Current frame rate: {self.frame_rate}")
            if self.frame_rate == 30:
                lte_data = {'signal_strength': 'good'}
                lte_message = json.dumps(lte_data)
                try:
                    result = self.client.publish(self.lte_topic, lte_message)
                    if result.rc != mqtt.MQTT_ERR_SUCCESS:
                        print(f"LTE publish failed with code: {result.rc}")
                        print(f"Reason: {mqtt.error_string(result.rc)}")
                except Exception as e:
                    print(f"Failed to publish LTE signal data: {e}")
                time.sleep(1)
            
            elif self.frame_rate == 20:
                lte_data = {'signal_strength': 'fair'}
                lte_message = json.dumps(lte_data)
                try:
                    result = self.client.publish(self.lte_topic, lte_message)
                    if result.rc != mqtt.MQTT_ERR_SUCCESS:
                        print(f"LTE publish failed with code: {result.rc}")
                        print(f"Reason: {mqtt.error_string(result.rc)}")
                except Exception as e:
                    print(f"Failed to publish LTE signal data: {e}")
                time.sleep(2)
            elif self.frame_rate == 10:
                print("LTE signal publishing disabled due to low FPS.")
            time.sleep(1)

    def publish_audio(self):
        print("Publishing audio data to topic: /drone/audio")
        pyaudio_instance = pyaudio.PyAudio()
        stream = pyaudio_instance.open(format=self.audio_format, channels=self.channels, rate=self.sample_rate, input=True, frames_per_buffer=self.chunk_size)

        while True:
            audio_data = stream.read(self.chunk_size)
            try:
                result = self.client.publish(self.audio_topic, audio_data)
                if result.rc != mqtt.MQTT_ERR_SUCCESS:
                    print(f"Audio publish failed with code: {result.rc}")
            except Exception as e:
                print(f"Error publishing audio: {e}")
            time.sleep(0.1)

# Initialize StreamPublisher class
publisher = StreamPublisher(topic="/drone/video", gps_topic="/drone/gps", lte_topic="/drone/lte_signal", audio_topic="/drone/audio")
