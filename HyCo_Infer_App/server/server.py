from server_read import RW_Server
from server_ws import WS_Server

import asyncio
import time

class Hyco_Integrator:

    def __init__(self, option: str = "ws", image_dir: str = "image", image_path: str = "/home/root/15.jpg"):
        self.option = option
        if self.option == "ws":
            self.processor = WS_Server(image_dir=image_dir)
            
        elif self.option == "rw":            
            self.processor = RW_Server(img_path=image_path)
        else:
            raise ValueError("Invalid option. Use 'ws' for WebSocket or 'rw' for Read/Write server.")
        self.start()

    
    def get_bboxes(self):
        if self.option == "ws":
            return asyncio.get_event_loop().run_until_complete(self.processor.process_images())
        elif self.option == "rw":
            start_time = time.time()
            print(f"--- Cycle {cycle} ---")
            # input_file = INP1 
            # print(f"Using input: {input_file.name}")        
            self.processor.write_input_files()        
            print("Waiting for CSV in output folder...")
            bbox = self.processor.wait_process_csv()
            print(f" {bbox} ")
            time_taken = time.time() - start_time
            print(f"Total time for cycle {cycle}: {time_taken:.2f} seconds")
            
            print("Clearing output folder...")
            self.processor.clear_folder(Hyco.output_folder)
            cycle =cycle +1
            return bbox
        else:
            raise ValueError("Invalid option. Use 'ws' for WebSocket or 'rw' for Read/Write server.")
        

def main():
    WS = Hyco_Integrator(option="ws", image_dir="image")
    bbox = WS.get_bboxes()
    print(bbox)