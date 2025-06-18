import os
import time
import json
import cv2

# Folder paths
FOLDER_PATH = "/home/root/client_json/"
FOLDER_PATH_img = "/home/root/images/"

SRC_IMAGE_PATH = "/home/root/client/racing.jpg"  # Source image

# Ensure directory exists
os.makedirs(FOLDER_PATH, exist_ok=True)

def create_image(i = 0):
    """Copies the source image to the target folder."""
    img = cv2.imread(SRC_IMAGE_PATH)  # Load image
    if img is None:
        print("Error: Source image not found!")
        return
    IMAGE_PATH = os.path.join(FOLDER_PATH_img, "racing" + str(i) + ".jpg")
    cv2.imwrite(IMAGE_PATH , img)  # Save the image in the monitoring folder
    print(f"Image saved at: {IMAGE_PATH}")

def process_detections():
    
    """Continuously monitors the folder for any JSON file and processes it."""
    while True:
        overall_start_time = time.time() 
        json_files = [f for f in os.listdir(FOLDER_PATH) if f.endswith(".json")]  # Find all JSON files

        if json_files:
            json_file_path = os.path.join(FOLDER_PATH, json_files[0])  # Pick the first JSON file
            print(f"Processing {json_file_path}...")

            with open(json_file_path, "r") as file:
                detections = json.load(file)
            os.remove(json_file_path)
            print(f"Deleted {json_file_path}. Waiting for next update...")

            create_image()
            print(json.dumps(detections, indent=4))
            # Process detections
            for obj in detections.get("objects", []):
                x, y, w, h = obj["x"], obj["y"], obj["width"], obj["height"]
                label = obj["label"]

                print(f"Detected: {label} at (x={x}, y={y}, w={w}, h={h})")
            if (0):
                break
            # Delete JSON file after processing
        time.sleep(1)
        overall_end_time = time.time()  # End overall execution timer
        print(f"Overall Execution Time for the communication : {overall_end_time - overall_start_time:.2f} seconds")
        print("Looking for json...")  # Check every second

if __name__ == "__main__":
    
    overall_start_time = time.time()  # Start overall execution timer
        
    for i in range(12):
            create_image(i)

    process_detections()

    overall_end_time = time.time()  # End overall execution timer
    print(f"Overall Execution Time: {overall_end_time - overall_start_time:.2f} seconds")
