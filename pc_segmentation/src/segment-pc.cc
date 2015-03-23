#include "common.h"
#include "bundler-parser.h"
#include "pc-handler.h"

using namespace bundler_parser;
using namespace pc_handler;

int main(int num_arguments, char** arguments) {
  
  google::InitGoogleLogging(arguments[0]);
  google::ParseCommandLineFlags(&num_arguments, &arguments, true);
  google::InstallFailureSignalHandler();
  
  BundlerParser parser(FLAGS_file);
  PcHandler pc_handle;
  
  std::vector<int> point_indices;
  for (int i = 1; i < 1540786; ++i) {
    point_indices.push_back(i);
  }
  std::vector<BundlerParser::Point3D> points = parser.get3dPoints(point_indices);
  
  for (auto point : points) {
    // pack r/g/b into rgb
    uint8_t r = point.color.at(0);
    uint8_t g = point.color.at(1);
    uint8_t b = point.color.at(2);

    // position
    pc_handle.point.x = point.position.at(0);
    pc_handle.point.y = point.position.at(1);
    pc_handle.point.z = point.position.at(2);
    
    // color
    pc_handle.point.r = r;
    pc_handle.point.g = g;
    pc_handle.point.b = b;
    
    // cloud
    pc_handle.cloud.push_back(pc_handle.point);
  }
  
  // visualize
  pcl::visualization::CloudViewer cloud_viewer("point cloud");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pointer(&pc_handle.cloud);
  cloud_viewer.showCloud(cloud_pointer, "point cloud");

  while (! cloud_viewer.wasStopped()) {
  }

  return 0;
}
