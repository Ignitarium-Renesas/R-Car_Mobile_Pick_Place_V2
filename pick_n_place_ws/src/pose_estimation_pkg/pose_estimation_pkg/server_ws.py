import asyncio
import websockets
import cv2
import numpy as np
import os
from pathlib import Path
import sys
import struct
import time


class WS_Server:
    def __init__(self, uri="ws://192.168.40.107:9091", image_dir="server/image"):
        self.uri = uri
        self.image_dir = image_dir

    @staticmethod
    def binary_to_floats(binary_data):
        num_floats = len(binary_data) // 4
        floats = struct.unpack('f' * num_floats, binary_data)
        return floats

    async def send_image(self, websocket, image_path):
        # Read image file
        img = cv2.imread(str(image_path))
        x_resize_ratio = 640 / img.shape[1]
        y_resize_ratio = 640 / img.shape[0]
        # Resize
        img_resized = cv2.resize(img, (640, 640))
        bgr_buff = img_resized.tobytes()
        starttime = time.time()
        # Send binary data over web sockets 'WebSocket'
        await websocket.send(bgr_buff)
        # wait text
        print("Time taken for send ", time.time() - starttime)
        binary = await websocket.recv()
        print("Time taken for inf ", time.time() - starttime)
        floats = self.binary_to_floats(binary)
        all = []
        print(all)
        for i in range(0, len(floats), 6):
            x1 = int(floats[i] / x_resize_ratio)
            y1 = int(floats[i + 1] / y_resize_ratio)
            x2 = int(floats[i + 2] / x_resize_ratio)
            y2 = int(floats[i + 3] / y_resize_ratio)
            confidence = floats[i + 4]
            class_ = int(floats[i + 5])
            print("bbox", floats[i], floats[i + 1], floats[i + 2], floats[i + 3])
            all.append([class_, confidence, x1, y1, x2, y2])
            
        return all
    async def process_images(self):
        print("Starting WebSocket server...")
        async with websockets.connect(self.uri) as websocket:
            print("Connected to WebSocket server at", self.uri)
            image_paths = list(Path(self.image_dir).glob("*"))
            print("Found images:", image_paths)
            bbox = None
            for image_path in image_paths:
                    starttime = time.time()
                    bbox = await self.send_image(websocket, image_path)
                    print("Time taken for processing:", time.time() - starttime)
            return bbox

            
if __name__ == "__main__":
    processor = WS_Server()
    print(asyncio.get_event_loop().run_until_complete(processor.process_images()))
