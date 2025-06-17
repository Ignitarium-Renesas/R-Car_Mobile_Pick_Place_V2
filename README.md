# Rcar_main_demo2

## Getting started

### Launch Demo GUI 

### Step 1. Open project Rover Robot Folder 
     
     cd ~/rcar_main_demo2/Project-Rover-Robot
     
###  Step 2. Install GUI 
     
     npm i
     
###  Step 3. Run GUI 
     
     node index.js

###  Step 4. Open browser & run 
     
     http://localhost:5000/

### Run 3D Pose Estimation
- Open 5 terminals in local and connect to IGN_Robo_5G

#### In 1st Terminal
```bash
ssh er@192.168.146
```
then run
```bash
ros2 launch realsense2_camera rs_launch.py
```
#### In 2nd Terminal
```bash
ssh er@192.168.146
```
then run
```bash
python3 Server_280.py
```
#### In 3rd Terminal
```bash
ssh root@192.168.0.217

docker start rcar
docker exec -it rcar bash
ros2 launch rcar_demo run_demo.launch.py
```

#### In 4th Terminal
```bash
ssh root@192.168.0.217

cd app_temp
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/lib:/home/root/app_temp/onnxruntime-linux-aarch64-1.17.1/lib:/home/root/app_temp

./rcar_app_ws yolo_v5s_ign-app/exec_config.json
```

#### In 5th Terminal
```bash
ssh root@192.168.0.217

docker exec -it rcar bash
ros2 param set /rcar_demo_node start_demo True
```