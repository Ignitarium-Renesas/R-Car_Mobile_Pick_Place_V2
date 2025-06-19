#include <tvm/runtime/module.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <thread>
#include <filesystem>
#include "include/json.hpp"
#include "independant_type.h"
#if !defined(DISABLE_ONNXRUNTIME)
#include "onnx_block.h"
#endif /* DISABLE_ONNXRUNTIME */
#include "prepostproc.h"
#include "rcar-xos/impfw/r_impfw.h"
#include "rcar-xos/osal/r_osal.h"
#include "rcar_module.h"
#include "stream_helpers.h"
#include <independant_type.h>

using json = nlohmann::json;

#include <iostream>
static void copy_image_to_input_buffer_and_quantize(void* input, const cv::Mat& normalized_image,
  int width, int height, int channels,
  const quantization_parameters& qp) {
std::cout << "Setting input" << std::endl;
float* image_data = reinterpret_cast<float*>(normalized_image.data);
int8_t* input_data = reinterpret_cast<int8_t*>(input);
for (int i = 0; i < height; ++i) {
for (int j = 0; j < width; ++j) {
for (int k = 0; k < channels; ++k) {
float res =
(image_data[i * width * channels + j * channels + k] / qp.scale_factor) + qp.zero_point;
input_data[k * width * height + i * width + j] = res;
}
}
}
}

static void _preprocess_input(cv::Mat& normalized_image, const cv::Mat& image, int width,
int height) {
cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
image.convertTo(normalized_image, CV_32F, 1.0 / 255.0);
}

void preprocess_input_data1(const int input_idx, void* input, tvm::runtime::ShapeTuple shape,
  IndependantType dtype, const quantization_parameters& qp) {
std::cout << "Set data ";
std::string shape_str = "";
int channels = shape[1];
int height = shape[2];
int width = shape[3];
for (int shape_idx = 0; shape_idx < shape.size(); shape_idx++) {
shape_str += std::to_string(shape[shape_idx]) + "x";
}
std::cout << "(" << shape_str << "): " << dtype << std::endl;

std::string folderPath = "/home/root/images/";
std::string filePath = " ";
namespace fs = std::filesystem;
while (true) {
  bool imageFound = false;
  for (const auto& entry : fs::directory_iterator(folderPath)) {
      if (entry.is_regular_file()) {
          filePath = entry.path().string();
          std::cout << "Image found: " << filePath << std::endl;
          
          imageFound = true;
          
          break; 
      }
  }
  if (!imageFound) {
      std::cout << "No image found, checking again..." << std::endl;
  }
  else{break;}
  std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait before checking again
}

const cv::Mat image = cv::imread(filePath);
std::remove(filePath.c_str());


//std::cout << "save tmep.png" << std::endl;
//cv::imwrite("temp.png", image);
cv::Mat normalized_image;
_preprocess_input(normalized_image, image, width, height);
copy_image_to_input_buffer_and_quantize(input, normalized_image, width, height, channels, qp);  
}

// typedef websocketpp::server<websocketpp::config::asio> server;

static std::tuple<std::chrono::high_resolution_clock::time_point,
                  std::chrono::high_resolution_clock::time_point>
start_inference(std::vector<st_osal_thread_config_t>& conf_thread, bool warmup,
                std::vector<std::vector<OutputInfo>>& outputs,
                std::vector<osal_thread_handle_t>& handle_thread,
                std::vector<int64_t>& return_value_thread,
                std::vector<RcarModule>& modules,
                const json& json_data,
                std::map<std::string, int>& mappings) {
                  
  auto start = std::chrono::high_resolution_clock::now();
  std::mutex mtx; 
  // server api_server;
  float qp = 0;
  tvm::runtime::ShapeTuple vshape;
  IndependantType vdtype;
  int key = -1;
  
  // get qp.
  while(1){
  quantization_parameters input_qp{.scale_factor = 1., .zero_point = 0};
  bool set_input = false;
  int to_input_idx = 0;
  for (auto& [to_key, to_model] : json_data["models"].items()) {
    to_input_idx = 0;
    for (auto& input : to_model["inputs"]) {
      std::string from_model = input["connection"][0];
      if (from_model == "-1") {
        if (input.find("quant_params") != input.end()) {
          auto qp = input["quant_params"];
          assert(qp.size() == 2);
          input_qp = {.scale_factor = qp[0], .zero_point = qp[1]};
        }
        key = mappings[to_key];
        vshape = modules[key].get_input_shape(to_input_idx);
        vdtype = modules[key].get_input_type(to_input_idx);

        void* data = reinterpret_cast<void*>(
          reinterpret_cast<int8_t*>(modules[key].get_input(to_input_idx)));
        set_input = true;
        break;
      }
    }
    if(set_input){
      break;
    }
    to_input_idx++;
  }

  std::cout << "input_qp:" << input_qp.scale_factor << ","<< input_qp.zero_point << std::endl;

  assert(set_input);
        mtx.lock();

        void* model_input_ptr = reinterpret_cast<void*>(
          reinterpret_cast<int8_t*>(modules[key].get_input(to_input_idx)));
        
        preprocess_input_data1(-1, model_input_ptr, vshape, vdtype, input_qp);
 
        int model_idx = 1;
        std::cout << "start inference with sent data" << std::endl;
        R_OSAL_ThreadCreate(&conf_thread[model_idx], 0x5003 + model_idx, &handle_thread[model_idx]);
        R_OSAL_ThreadJoin(handle_thread[0+model_idx], &return_value_thread[0+model_idx]);
        std::cout << "parse outputs" << std::endl;
        std::vector<OutputInfo>& output_infos = outputs[model_idx];


        for (int output_idx = 0; output_idx < output_infos.size(); ++output_idx) {
          OutputInfo& output_info = output_infos[output_idx];
          const int output_idx_in_model = output_info.idx_in_model;
          output_info.shape = output_info.module->get_output_shape(output_idx_in_model);
          output_info.type = output_info.module->get_output_type(output_idx_in_model);
          output_info.ptr = output_info.module->get_output(output_idx_in_model);

  
          mtx.unlock();
        }
        postprocess_output(output_infos); 
        
      }

  
 auto end = std::chrono::high_resolution_clock::now();
  return {start, end};
}

int64_t run_in_thread1(void* ptr) {
  thread_params* tp = reinterpret_cast<thread_params*>(ptr);
  RcarModule* module = reinterpret_cast<RcarModule*>(tp->module);
  module->run();
  return 0;
}

static void setup_outputs(const json& config, std::vector<RcarModule>& modules,
                          std::vector<std::vector<OutputInfo>>& outputs,
                          std::map<std::string, int>& mappings) {
  int model_idx = 0;
  int global_output_idx = 0;
  outputs.resize(config["model_outputs"].size());
  for (auto& from_model : config["model_outputs"]) {
    std::vector<OutputInfo>& model_out = outputs[model_idx];
    for (auto& connection : from_model) {
      std::string output_model = connection["connection"][0];
      int output_idx = connection["connection"][1];

      std::string output_runtime = config["models"][output_model]["runtime"];
      int idx = mappings[output_model];
      RcarModule& module = modules[idx];

      quantization_parameters output_qp{.scale_factor = 1., .zero_point = 0};

      if (connection.find("quant_params") != connection.end()) {
	auto qp = connection["quant_params"];
        assert(qp.size() == 2);
        output_qp = {.scale_factor = qp[0], .zero_point = qp[1]};
      }

      OutputInfo oi{.module = &modules[idx],
                    .qp = output_qp,
                    .idx_in_model = output_idx,
                    .global_idx = global_output_idx};

      model_out.push_back(oi);
      global_output_idx++;
    }
    model_idx++;
  }
}

static void setup_threadparams(std::vector<thread_params>& thread_params,
                               std::vector<st_osal_thread_config_t>& conf_thread,
                               std::vector<RcarModule>& modules) {
  for (int module_idx = 0; module_idx < modules.size(); ++module_idx) {
    thread_params[module_idx].id = module_idx;
    thread_params[module_idx].module = &modules[module_idx];

    conf_thread[module_idx] = {run_in_thread1, &thread_params[module_idx],
                               OSAL_THREAD_PRIORITY_TYPE10, "1" + module_idx, 0x4000};
  }
}

static void connect_models(const json& data, std::vector<RcarModule>& modules,
                           std::map<std::string, int>& mappings) {
  for (auto& [to_key, to_model] : data["models"].items()) {
    std::string to_runtime = to_model["runtime"];

    /* nohlmann json doesn't seem to support std::distance, so keeping track of the id manually */
    int to_input_idx = 0;
    for (auto& input : to_model["inputs"]) {
      std::string from_model = input["connection"][0];

      std::cout << "From model " << from_model << std::endl;
      /* need to load from a file */
      if (from_model == "-1") {
        quantization_parameters input_qp{.scale_factor = 1., .zero_point = 0};

        if (input.find("quant_params") != input.end()) {
          auto qp = input["quant_params"];
          assert(qp.size() == 2);
          input_qp = {.scale_factor = qp[0], .zero_point = qp[1]};
        }
        if (to_runtime == "tvm") {
          std::string input_path = input["connection"][1];
          std::cout << "Setting input " << to_key << ":" << +to_input_idx << " with " << input_path
                    << std::endl;

          tvm::runtime::ShapeTuple vshape = modules[mappings[to_key]].get_input_shape(to_input_idx);
          IndependantType vdtype = modules[mappings[to_key]].get_input_type(to_input_idx);

          void* data = reinterpret_cast<void*>(
              reinterpret_cast<int8_t*>(modules[mappings[to_key]].get_input(to_input_idx)));
          preprocess_input(to_input_idx, data, vshape, vdtype, input_path.c_str(), input_qp);
        } else {
          assert(false);

        }
      } else {
        int from_output_idx = input["connection"][1];
        std::string from_runtime = data["models"][from_model]["runtime"];

        std::cout << "Connecting from " << input << " type: " << from_runtime << std::endl;
        std::cout << "Connecting to " << to_model << ":" << to_input_idx << " type: " << to_runtime
                  << std::endl;

        modules[mappings[to_key]].set_input(from_output_idx, to_input_idx,
                                            modules[mappings[from_model]]);
      }
      to_input_idx++;
    }
  }

  std::cout << "Done connecting" << std::endl;
}

static void load_models(const json& data, std::vector<RcarModule>& modules,
                        std::map<std::string, int>& mappings) {
  int model_idx = 0;
  for (auto& [key, model] : data["models"].items()) {
    std::string runtime = model["runtime"];
    std::cout << "Registering model " << key << std::endl;
    if (runtime == "tvm") {
      modules.push_back(RcarModule(TVM, model["input_model"], true));
    } else if (runtime == "onnx") {
#if !defined(DISABLE_ONNXRUNTIME)
        modules.push_back(RcarModule(ONNX, model["input_model"],true));
#endif  
    } else {
      std::cerr << "Invalid runtime type for " << model << std::endl;
      exit(-1);
    }
    mappings.insert({key, model_idx});
    model_idx++;
  }
}

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <configuration file>" << std::endl;
    return -1;
  }

  std::vector<st_osal_thread_config_t> conf_thread;
  std::vector<thread_params> thread_params;
  std::vector<osal_thread_handle_t> handle_thread;
  std::vector<int64_t> return_value_thread;

  const char* const config_path = argv[1];
  std::ifstream f(config_path);
  json json_data = json::parse(f);

  const int loop_count = json_data["app"]["number_of_loops"];
  const int frame_per_loop = json_data["app"]["frame_per_loop"];
  const int warmup = json_data["app"]["warmup"];

  std::vector<std::vector<OutputInfo>> outputs;
  std::map<std::string, int> mappings;

  std::vector<RcarModule> modules;
  load_models(json_data, modules, mappings);

  thread_params.resize(modules.size());
  conf_thread.resize(modules.size());
  handle_thread.resize(modules.size());
  return_value_thread.resize(modules.size());

  setup_threadparams(thread_params, conf_thread, modules);

  connect_models(json_data, modules, mappings);
  setup_outputs(json_data, modules, outputs, mappings);

  std::cout << "Running model" << std::endl;

  auto [start, end] = start_inference(conf_thread, warmup, outputs, handle_thread,
                                      return_value_thread, modules, json_data, mappings);
  auto total_exec_time = static_cast<double>((end - start).count()) / 1e6;
  auto throughput = total_exec_time / (loop_count * frame_per_loop);
  std::cout << "Total exec time (ms): " << total_exec_time << std::endl;
  std::cout << "Loop count: " << loop_count * frame_per_loop << std::endl;
  std::cout << "Throughput inv. (ms): " << throughput << std::endl;
  std::cout << "Throughput (fps): " << 1000 / throughput << std::endl;

  return 0;
}
