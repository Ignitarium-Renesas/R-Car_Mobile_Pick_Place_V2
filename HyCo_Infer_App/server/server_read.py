import os
import time
import shutil
from pathlib import Path
import pandas as pd
import subprocess
import cv2
import argparse
# Folder paths




class RW_Server():

    def __init__(self, img_path: Path = Path("/home/root/15.jpg")):        
        self.set_paths()
        self.img_path = img_path
        
    def set_paths(self):    
        self.input_folder = Path("/home/root/app_temp/yolo_v7t-app/test_data_img")
        self.output_folder = Path("/home/root/app_temp/outputs")
        self.output_file = Path("/home/root/app_temp/outputs/output.csv")
        self.awknowlegement_file = "/home/root/app_temp/t2.txt"
        self.hand_shake_file = "/home/root/app_temp/t1.txt"


    def trigger_external_program(self, debug: str = "False"):
        """Launch the external program independently."""
        if debug == False:
            subprocess.Popen(
                ["./rcar_app", "./yolo_v7t-app/exec_config.json"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
                cwd="/home/root/app_temp"
            )
        else:
            subprocess.Popen(
            ["./rcar_app", "./yolo_v7t-app/exec_config.json"],
            # stdout=subprocess.DEVNULL,
            # stderr=subprocess.DEVNULL,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            cwd="/home/root/app_temp"
            )

    def clear_folder(self, folder: Path):
        for item in folder.glob("*"):
            if item.is_file():
                item.unlink()
            elif item.is_dir():
                shutil.rmtree(item)

    def write_img_to_file_Location(self, img, target_path):
        cv2.imwrite(img, target_path)
    
    def write_path_to_file_location(self,file, target_path):
        shutil.copy(file, target_path)


    def write_input_files(self, n: int = 12):
        """Clears input folder and writes n copies of source_file"""
        if not self.input_folder.exists():
            self.input_folder.mkdir(parents=True)
        self.clear_folder(self.input_folder)  
        
        with open(self.hand_shake_file, "w") as file:
            file.write("yes")  # Create an empty file
        source_file = self.img_path
        if not source_file.exists():
            raise FileNotFoundError(f"Source file {source_file} does not exist.")
        print(f"File '{self.hand_shake_file}' created successfully.")
        
        for i in range(n):
            target = self.input_folder / f"file_{i+1}{source_file.suffix}"
            self.write_path_to_file_location(source_file, target)
        
        return None

    def convert_bbox_to_original(self, bboxes, original_width = 848, original_height = 480, resized_width = 640, resized_height = 640):
        """
        Converts bounding boxes from resized dimensions back to original dimensions.

        :param bboxes: List of bounding boxes in the format [Label, X1, Y1, X2, Y2, Score]
        :param original_width: Width of the original image
        :param original_height: Height of the original image
        :param resized_width: Width of the resized image
        :param resized_height: Height of the resized image
        :return: List of bounding boxes in the original dimensions
        """
        scale_x = original_width / resized_width
        scale_y = original_height / resized_height

        original_bboxes = []
        for bbox in bboxes:
            label, x1, y1, x2, y2, score = bbox
            x1_original = x1 * scale_x
            y1_original = y1 * scale_y
            x2_original = x2 * scale_x
            y2_original = y2 * scale_y
            original_bboxes.append([label, x1_original, y1_original, x2_original, y2_original, score])

        return original_bboxes

    def wait_process_csv(self):
        """Waits for a CSV file in the output folder, returns time and path"""
        if not self.output_folder.exists():
            self.output_folder.mkdir(parents=True)

        start_time = time.time()
        csv_path = None
        
        while True:
            if os.path.exists(self.awknowlegement_file):
                print(f"File found: {self.awknowlegement_file}")
                os.remove(self.awknowlegement_file)
                print(f"File removed: {self.awknowlegement_file}")
                for file in self.output_folder.glob("*.csv"):
                    csv_path = file
                    break
                break
            # print("Waiting for acknowledgment file...")
            time.sleep(0.01)

        read_time = time.time()
        df = pd.read_csv(csv_path)
        print("\n=== CSV FILE CONTENT ===")
        print(df.to_string(index=False))
        # print("========================\n")

        return self.convert_bbox_to_original(df.values.tolist())

def main():

    # INP1 = Path("/home/root/15.jpg")
    parser = argparse.ArgumentParser(description="Hyco")    
    parser.add_argument("-t","--trigg", type=str, help="Trigger Program or Not", default = "True")
    parser.add_argument("-d","--debug", type=str, help="Trigger Debug or Not", default = "False")
    args = parser.parse_args()
    cycle = 1
    Hyco = RW_Server()
    if args.trigg == "True":
        Hyco.trigger_external_program(args.debug)
    while True:
        start_time = time.time()
        print(f"--- Cycle {cycle} ---")
        # input_file = INP1 
        # print(f"Using input: {input_file.name}")        
        Hyco.write_input_files()        
        print("Waiting for CSV in output folder...")
        bbox = Hyco.wait_process_csv()
        print(f" {bbox} ")
        time_taken = time.time() - start_time
        print(f"Total time for cycle {cycle}: {time_taken:.2f} seconds")
        
        print("Clearing output folder...")
        Hyco.clear_folder(Hyco.output_folder)
        cycle =cycle +1



if __name__ == "__main__":
    main()

