#include "common.h"
#include "bundler-parser.h"

using namespace bundler_parser;

int main(int num_arguments, char** arguments) {
  
  google::InitGoogleLogging(arguments[0]);
  google::ParseCommandLineFlags(&num_arguments, &arguments, true);
  google::InstallFailureSignalHandler();
  
  BundlerParser parser(FLAGS_file);
  
  // Indices
  std::vector<int> indices;
  std::string::size_type next;
  std::string temp_string = FLAGS_indices;
  while (! temp_string.empty()) {
    indices.push_back(std::stoi(temp_string, &next));
    temp_string = temp_string.substr(next);
  }
  
  if (FLAGS_query == "camera") {
    std::vector<BundlerParser::Camera> cameras = parser.getCameras(indices);
    
    int index = 0;
    for (auto camera : cameras) {
      parser.printCamera(camera);
      LOG(INFO) << "--------------";
      ++index;
    }
  }
  
  if (FLAGS_query == "3dpoint") {
    std::vector<BundlerParser::Point3D> points = parser.get3dPoints(indices);
    
    int index = 0;
    for (auto point : points) {
      parser.print3DPoint(point);
      LOG(INFO) << "--------------";
      ++index;
    }
  }
  
  return 0;
}
