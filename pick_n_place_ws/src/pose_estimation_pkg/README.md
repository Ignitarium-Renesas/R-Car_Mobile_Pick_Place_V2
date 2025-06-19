# pose_estimation_3dcam

Pose estimation module for object picking in R_car.

## Clone
ssh into rcar board

```bash
#Inside Docker in RZV4H
git config --global credential.helper 'cache --timeout=3600'
git clone --recursive https://gitlab.ignitarium.in/ign-ai/customer/renesas/rcar/Pose_Estimate_Pkg.git --depth=1
```

## Installation
### Install ros2
Refer this link to install [ros2 humble](https://docs.ros.org/en/humble/Installation.html).

### Linux (PIP Based)

```bash
cd Pose_Estimate_Pkg
python -m pip install -U pip

pip install -r requirements.txt
```

### Run
```bash
# Build the package
cd ..
colcon build
source install/setup.bash 
```

#### Run service node

```bash
ros2 run pose_estimation_pkg camera_pose_srv
```

#### Run Web Socket Application
```bash
#Inside RZV4H
cd app_temp
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/lib:/home/root/app_temp/onnxruntime-linux-aarch64-1.17.1/lib:/home/root/app_temp

./rcar_app_ws yolo_v5s_ign-app/exec_config.json
```
#### Run client

```bash
ros2 run pose_estimation_pkg pose_est_client
```