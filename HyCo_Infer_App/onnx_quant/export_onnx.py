import torch
import cv2
import numpy as np

from ultralytics import YOLO


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="Input your yolo model.")
    parser.add_argument("--format", type=str,default="onnx", help="Path to input image.")
    parser.add_argument('--dynamic', action='store_true', help='dynamic ONNX axes')
    args = parser.parse_args()

    model = YOLO(args.model)
    model.export(format=args.format, dynamic=args.dynamic)
