from pose_estimation_pkg.server_read import RW_Server
from pose_estimation_pkg.server_ws import WS_Server

import asyncio
import time

class Hyco_Integrator:

    def __init__(self, option: str = "ws", image_dir: str = "/home/baveshs/HYcodemo/HyCo_Demo/server/image", image_path: str = "/home/root/15.jpg"):
        self.option = option
        if self.option == "ws":
            self.processor = WS_Server(image_dir=image_dir)
            
        elif self.option == "rw":            
            self.processor = RW_Server(img_path=image_path)
        else:
            raise ValueError("Invalid option. Use 'ws' for WebSocket or 'rw' for Read/Write server.")

    
    async def get_bboxes(self):
        if self.option == "ws":
            return await self.processor.process_images()
        elif self.option == "rw":
            start_time = time.time()
            print(f"--- Cycle {cycle} ---")
            self.processor.write_input_files()        
            print("Waiting for CSV in output folder...")
            bbox = self.processor.wait_process_csv()
            print(f" {bbox} ")
            time_taken = time.time() - start_time
            print(f"Total time for cycle {cycle}: {time_taken:.2f} seconds")
            
            print("Clearing output folder...")
            self.processor.clear_folder(self.processor.output_folder)
            cycle =cycle +1
            return bbox
        else:
            raise ValueError("Invalid option. Use 'ws' for WebSocket or 'rw' for Read/Write server.")

if __name__ == "__main__":
    WS = Hyco_Integrator(option="ws", image_dir="image")
    bbox = WS.get_bboxes()
    print(bbox)