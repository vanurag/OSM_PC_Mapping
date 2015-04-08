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
  
  // get camera data
  std::vector<int> camera_indices;
  for (int i = 0; i < parser.num_images; ++i) {
    camera_indices.push_back(i);
  }
  pc_handle.cameras = parser.getCameras(camera_indices);
  
  // get 3d point data
  std::vector<int> point_indices;
  for (int i = 0; i < parser.num_points; ++i) {
    point_indices.push_back(i);
  }
  std::vector<BundlerParser::Point3D> points = parser.get3dPoints(point_indices);
  
  // register the parsed 3d points to cloud
  pc_handle.addToCloud(points);
  
  // mean-adjusted cloud
  pc_handle.meanAdjustCloud(pc_handle.cloud);

  // mean-adjusted cameras
  pc_handle.meanAdjustCameras(pc_handle.cameras);

  // Ground Plane
  pc_handle.estimateGroundPlane();

  // Point Normals
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pointer(&pc_handle.cloud);
  pc_handle.estimateNormals(cloud_pointer, 0.4);

  // visualize
  pc_handle.visualize(FLAGS_show_cloud, FLAGS_show_cameras);
  

  return 0;
}
