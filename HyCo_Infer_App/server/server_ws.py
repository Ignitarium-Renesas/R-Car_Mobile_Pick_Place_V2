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
    def __init__(self, uri="ws://localhost:9002", image_dir="image"):
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
        for i in range(0, len(floats), 6):
            x1 = int(floats[i] / x_resize_ratio)
            y1 = int(floats[i + 1] / y_resize_ratio)
            x2 = int(floats[i + 2] / x_resize_ratio)
            y2 = int(floats[i + 3] / y_resize_ratio)
            confidence = floats[i + 4]
            class_ = int(floats[i + 5])
            print("bbox", floats[i], floats[i + 1], floats[i + 2], floats[i + 3])
            print("confidence", floats[i + 4], "class", floats[i + 5])
            l = [class_, confidence, x1, y1, x2, y2]
            all_ = all.append(l)

            # if confidence > 0.6:
            #     cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            #     cv2.putText(img, "Conf: " + f"{confidence:.2f}" + ", Class: " + str(class_),
            #                 (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # cv2.imwrite("bbox_" + os.path.basename(image_path), img)
        return all_

    async def process_images(self):
        if len(sys.argv) > 1:
            print("WebSocket URI and port:", sys.argv[1:])
            self.uri = "ws://" + sys.argv[1]
        else:
            print(f"WebSocket URI is not set, using default: {self.uri}")

        async with websockets.connect(self.uri) as websocket:
            image_paths = list(Path(self.image_dir).glob("*"))

            for image_path in image_paths:
                    print("send:", image_path)
                    starttime = time.time()
                    bbox = await self.send_image(websocket, image_path)
                    print("Time taken for processing:", time.time() - starttime)
                    await asyncio.sleep(1)
                    print("Time taken for processing:", time.time() - starttime)
            return bbox


if __name__ == "__main__":
    processor = WS_Server()
    print(asyncio.get_event_loop().run_until_complete(processor.process_images()))