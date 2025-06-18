import numpy as np
from onnxruntime.quantization import CalibrationDataReader, quantize_static, QuantType, QuantFormat
import cv2
import os

    
# Class for Callibration Data reading
class ImageCalibrationDataReader(CalibrationDataReader):
    def __init__(self, image_paths):
        self.image_paths = image_paths
        self.idx = 0
        self.input_name = "images"

    def preprocess(self, frame):
        # Same preprocessing that you do before feeding it to the model
        frame = cv2.imread(frame)
        X = cv2.resize(frame, (640, 640))
        image_data = np.array(X).astype(np.float32) / 255.0  # Normalize to [0, 1] range
        image_data = np.transpose(image_data, (2, 0, 1))  # (H, W, C) -> (C, H, W)
        image_data = np.expand_dims(image_data, axis=0)  # Add batch dimension
        return image_data

    def get_next(self):
        # method to iterate through the data set
        if self.idx >= len(self.image_paths):
            return None

        image_path = self.image_paths[self.idx]
        input_data = self.preprocess(image_path)
        self.idx += 1
        return {self.input_name: input_data}


if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, default="preprocessed.onnx", help="Input your onnx model.")
    parser.add_argument("--model_type", type=str,default="yolov8n", help="yolo model type")
    parser.add_argument("--out_path", type=str, default="static_quantized.onnx", help="output path to your onnx model.")
    parser.add_argument("--data_path", type=str,default="tiny_coco_dataset/tiny_coco/train2017", help="calibration data path")
    args = parser.parse_args()

    DATA_PATH = args.data_path
    # Assuming you have a list of image paths for calibration
    calibration_image_paths = [os.path.join(DATA_PATH,image) for image in os.listdir(DATA_PATH)]
    # calibration_image_paths = ['test.jpg'] # you can add more of the image paths

    # Create an instance of the ImageCalibrationDataReader
    calibration_data_reader = ImageCalibrationDataReader(calibration_image_paths)

    # get the preprocessed model.
    # !python -m onnxruntime.quantization.preprocess --input yolov8n.onnx --output preprocessed.onnx

    # Use the calibration_data_reader with quantize_static  

    if "yolov5" in args.model_type:
        nodes_to_exclude=['/model.24/Concat_3', '/model.24/Split', '/model.24/Sigmoid'
                         '/model.24/dfl/Reshape', '/model.24/dfl/Transpose', '/model.24/dfl/Softmax', 
                         '/model.24/dfl/conv/Conv', '/model.24/dfl/Reshape_1', '/model.24/Slice_1',
                         '/model.24/Slice', '/model.24/Add_1', '/model.24/Sub', '/model.24/Div_1',
                          '/model.24/Concat_4', '/model.24/Mul_2', '/model.24/Concat_5']
    elif "yolov8" in args.model_type:
        nodes_to_exclude=['/model.22/Concat_3', '/model.22/Split', '/model.22/Sigmoid'
                         '/model.22/dfl/Reshape', '/model.22/dfl/Transpose', '/model.22/dfl/Softmax', 
                         '/model.22/dfl/conv/Conv', '/model.22/dfl/Reshape_1', '/model.22/Slice_1',
                         '/model.22/Slice', '/model.22/Add_1', '/model.22/Sub', '/model.22/Div_1',
                          '/model.22/Concat_4', '/model.22/Mul_2', '/model.22/Concat_5'], 
    else:
        print("The models should be one of yolov8 or yolov5")
        
    quantize_static(args.model, args.out_path,
                    weight_type=QuantType.QInt8,
                    activation_type=QuantType.QUInt8,
                    calibration_data_reader=calibration_data_reader,
                    quant_format=QuantFormat.QDQ,
                    nodes_to_exclude=nodes_to_exclude,
                    per_channel=False,
                    reduce_range=True,)
    
    