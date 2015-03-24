#include "common.h"
#include "flags.h"
#include "pc-handler.h"

using namespace bundler_parser;
using namespace pc_handler;

int main(int num_arguments, char** arguments) {
  
  google::InitGoogleLogging(arguments[0]);
  google::ParseCommandLineFlags(&num_arguments, &arguments, true);
  google::InstallFailureSignalHandler();
  
  BundlerParser parser(FLAGS_bundler_file);
  PcHandler pc_handle;
  
  std::vector<int> point_indices;
  for (int i = 0; i < parser.num_points; ++i) {
    point_indices.push_back(i);
  }
  std::vector<BundlerParser::Point3D> points = parser.get3dPoints(point_indices);
  
  // register the parsed 3d points to cloud
  pc_handle.addToCloud(&points);
  
  // mean-adjusted cloud
  pc_handle.meanAdjustCloud();
  
  // visualize
  pc_handle.visualize();
  

  return 0;
}
