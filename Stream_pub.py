#!/usr/bin/env python
import cv2
import threading
import paho.mqtt.client as mqtt
import queue
import time

class Stream_publisher:
    
    def __init__(self, topic, video_address=0, start_stream=True, host="3.110.177.25", port=1883) -> None:
        self.client = mqtt.Client()  # create new instance
        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish
        self.client.connect(host, port)
        self.topic = topic
        self.video_source = video_address

        self.cam = cv2.VideoCapture(self.video_source)
        
        # Initial frame rate
        self.frame_rate = 10  # Target 10 frames per second
        self.prev_time = time.time()

        self.frame_queue = queue.Queue(maxsize=2)  # Limit the size to prevent excessive memory usage

        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.publish_thread = threading.Thread(target=self.publish_frames)

        self.publish_success_count = 0
        self.publish_total_count = 0

        if start_stream:
            self.capture_thread.start()
            self.publish_thread.start()

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")

    def on_publish(self, client, userdata, mid):
        self.publish_success_count += 1

    def start_streaming(self):
        if not self.capture_thread.is_alive():
            self.capture_thread = threading.Thread(target=self.capture_frames)
            self.capture_thread.start()
        if not self.publish_thread.is_alive():
            self.publish_thread = threading.Thread(target=self.publish_frames)
            self.publish_thread.start()

    def capture_frames(self):
        print("Capturing from video source: {}".format(self.video_source))
        while True:
            ret, img = self.cam.read()
            if not ret:
                print("Failed to read frame")
                break  # Exit if the video source is not available
            current_time = time.time()
            if (current_time - self.prev_time) >= (1. / self.frame_rate):
                self.prev_time = current_time
                if not self.frame_queue.full():
                    self.frame_queue.put(img)
                else:
                    # Drop the oldest frame if the queue is full
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put(img)
                    except queue.Empty:
                        pass

    def publish_frames(self):
        print("Publishing to topic: {}".format(self.topic))
        while True:
            if not self.frame_queue.empty():
                img = self.frame_queue.get()
                img_str = cv2.imencode('.jpg', img)[1].tobytes()
                try:
                    self.client.publish(self.topic, img_str)
                    self.publish_total_count += 1
                except Exception as e:
                    print(f"Failed to publish: {e}")
                
                # Adjust frame rate based on success rate
                if self.publish_total_count >= 10:  # Adjust every 10 frames
                    success_rate = self.publish_success_count / self.publish_total_count
                    if success_rate < 0.8:
                        self.frame_rate = max(3, self.frame_rate - 1)  # Decrease frame rate
                    elif success_rate > 0.9:
                        self.frame_rate = min(10, self.frame_rate + 1)  # Increase frame rate
                    print(f"Adjusted frame rate to: {self.frame_rate}")
                    self.publish_success_count = 0
                    self.publish_total_count = 0

if __name__ == "__main__":
    webcam = Stream_publisher("test", video_address=0)  # streaming from webcam (0) to topic: "test"
    # file = Stream_publisher("test", video_address="kungfu-panda.mkv")  # streaming from a file to topic: "test"