#include "prepostproc.h"

#include <cstdint>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <string>

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

void preprocess_input(const char* model_name, const int input_idx, void* input, tvm::runtime::ShapeTuple shape,
                      IndependantType dtype, const char* path_to_input_file,
                      const quantization_parameters& qp) {
  std::cout << "Loading image " << path_to_input_file;
  std::string shape_str = "";
  int channels = shape[1];
  int height = shape[2];
  int width = shape[3];
  for (int shape_idx = 0; shape_idx < shape.size(); shape_idx++) {
    shape_str += std::to_string(shape[shape_idx]) + "x";
  }
  std::cout << "(" << shape_str << "): " << dtype << std::endl;

  const cv::Mat image = cv::imread(path_to_input_file);
  cv::Mat normalized_image;
  _preprocess_input(normalized_image, image, width, height);
  copy_image_to_input_buffer_and_quantize(input, normalized_image, width, height, channels, qp);
}

static void writeFloatArrayToFile(const float* floatArray, int arraySize, std::string filename) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
    return;
  }

  const int numbersPerLine = 5;
  int count = 0;
  for (int i = 0; i < arraySize; ++i) {
    outFile << floatArray[i];
    count++;
    if (count % numbersPerLine == 0 || i == arraySize - 1) {
      outFile << std::endl;
    } else {
      outFile << ", ";
    }
  }

  outFile.close();
  // std::cout << "Output written to " << filename << std::endl;
}

static void writeIntArrayToFile(const int64_t* intArray, int arraySize, std::string filename) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
    return;
  }

  const int numbersPerLine = 1;
  int count = 0;
  for (int i = 0; i < arraySize; ++i) {
    outFile << intArray[i];
    count++;
    if (count % numbersPerLine == 0 || i == arraySize - 1) {
      outFile << std::endl;
    } else {
      outFile << ", ";
    }
  }

  outFile.close();
}

  static void callback_postprocess(std::vector<OutputInfo>& output_info) {
    int id = output_info[0].global_idx;
    int16_t shapenms = output_info[0].shape[1];
  
    int64_t* labels = reinterpret_cast<int64_t*>(output_info[1].ptr);
    float* dets = reinterpret_cast<float*>(output_info[0].ptr);
  
    std::string output_file = std::string("/home/root/output_bbox/output") + ".csv";

    std::ofstream csv_file(output_file);
    if (!csv_file.is_open()) {
      std::cerr << "Error: Unable to open file " << output_file << " for writing." << std::endl;
      return;
    }
  
    // Write header
    csv_file << "Label,BBox_X1,BBox_Y1,BBox_X2,BBox_Y2,Score\n";
  
    std::cout << "Bounding Boxes and Labels (id: " << id << "):" << std::endl;
    for (int i = 0; i < shapenms; ++i) {
      std::cout << "Box " << i + 1 << " (Label: " << labels[i] << "): ";
      csv_file << labels[i]; // write label to CSV
  
      for (int j = 0; j < 5; ++j) {
        float value = dets[i * 5 + j];
        std::cout << value;
        csv_file << "," << value;
        if (j < 4) std::cout << ", ";
      }
  
      std::cout << std::endl;
      csv_file << "\n";
    }
  
    csv_file.close();
    std::cout << "Results saved to " << output_file << std::endl;
  }  

void postprocess_output(const char* model_name, std::vector<OutputInfo>& output_info) {
  if (output_info.size() == 2) {
    callback_postprocess(output_info);
  } else {
    std::cerr << "Unexpected output_size " << +output_info.size() << ". Exiting" << std::endl;
    exit(1);
  }
}
