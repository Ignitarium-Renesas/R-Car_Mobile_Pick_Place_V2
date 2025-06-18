# WARNING: Ensure all environment variables and paths are correctly set before executing any commands.

NOTE: *Refer this [readme](./onnx_quant/README_ONNX_QDQ_QUANTIZATION.md) for onnx quantization*
# Part 1


1. Execute the base common application in Reaction to create the necessary files.
  Follow Chapter 14 of the HyCo Reaction UM 'R_Car_V4x_HyCo_L_REACTION_UserManual.pdf' to execute the common application in Reaction.
  Here, specify reaction.yaml as shown below and execute it.
'''
experiment:
  model_name: YOLO_v7t-app
  task: tvm_cch
  action: app
  line: onnx
  target: v4h2
  convert_configs:
    tvm:
      host: 192.168.0.20
      port: 9090
      keep_app_folder: true # to keep artifacts on board for rerun
      remove_input_quantize: True
      remove_output_dequantize: True
'''

2. Copies folder 'generator_cmake' to Ubuntu PC that was installed the R-Car SDK v3.33.0 and HyCo addon v20250109_rc4.
  ex.) ./reaction/app/generator_cmake

3. Change current directory to HyCo reaction folder as follows
'''
cd /opt/rcar-xos/v3.33.0/tools/hyco/reaction
'''

4. Copies necessary files to the folder 'generator_cmake' as follows
'''
cp ./work_dir/yolo_v7t-app/tvm-v4h2/prepostproc.cc ./reaction/app/generator_cmake
cp ./work_dir/yolo_v7t-app/tvm-v4h2/rcar_app.cc ./reaction/app/generator_cmake
'''

5. Export necessary environment variables as follows
'''
export CEVA_TOOLBOX_ROOT="/tmp"
export SNCSDT_LICENSE_FILE="license"
export DATA_DIR="/home/baveshs/Renesas/rcar-xos/v3.33.0/tools/hyco/reaction/data"
export RCAR_XOS_PATH="/home/baveshs/Renesas/rcar-xos/"
export CEVA_CSL_PATH="/opt/ceva_csl/"
'''

6. Launch docker as follows
'''
./dockerfiles/run.sh reaction/tvm-app-linux-v4h2-cpu:v0.1
'''

7. Change current directory to the cmake_folder as follows
'''
cd reaction/app/generator_cmake/
'''

8. Create symbolic link file to build the application as follows
'''
ln -s /opt/rcar-xos/v3.33.0/tools/toolchains/poky/sysroots/aarch64-poky-linux/lib/libc.so.6 /lib/libc.so.6
ln -s /opt/rcar-xos/v3.33.0/tools/toolchains/poky/sysroots/aarch64-poky-linux/usr/lib/libc_nonshared.a /usr/lib/libc_nonshared.a
'''

9. Execute the sh script file to build the application as follows
'''
./aarch_build.sh
'''

10. Confirm binary file of the application as follows
'''
ls -l ./build_aarch64/rcar_app_cmake 
-rwxr-xr-x 1 root root 1894816 Feb 14 01:59 ./build_aarch64/rcar_app_cmake
'''

# Part 2

1. SSH into two new terminals.

2. Compile the input file in the first SSH terminal:
'''
g++ -o sample_input sample_input.cc
'''

3. Compile the output file in the second SSH terminal:
'''
g++ -o sample_out sample_out.cc
'''

4. Execute the compiled input file:
'''
./sample_input
'''

5. Execute the compiled output file:
'''
./sample_out
'''

# Part 3

1. Copy the binary 'rcar_app_cmake' to the common application environment 'app_temp' as follows.
Execute the following command on the Host PC:
'''
scp /opt/rcar-xos/v3.33.0/tools/hyco/reaction/reaction/app/generator_cmake/build_aarch64/rcar_app_cmake root@192.168.0.20:/home/root/app_temp
'''

2. Execute the binary 'rcar_app_cmake' as follows:
'''
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/lib:/home/root/app_temp/onnxruntime-linux-aarch64-1.17.1/lib:/home/root/app_temp
./rcar_app_cmake ./multinets_v2_display-app/exec_config.json
'''

