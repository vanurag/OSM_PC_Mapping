#ifndef OSM_PC_MAPPING_PC_HANDLER_H_
#define OSM_PC_MAPPING_PC_HANDLER_H_

#include "common.h"

namespace pc_handler {

class PcHandler {
 public:
  PcHandler() {
    mean_point.push_back(0.0);
    mean_point.push_back(0.0);
    mean_point.push_back(0.0);
  };
  virtual ~PcHandler() {};

  // adds additional points to the cloud
  void addToCloud(
      std::vector<bundler_parser::BundlerParser::Point3D>& points);

  // adjust the point cloud such that it's mean is at the origin
  void meanAdjustCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud);

  // adjust the camera centers such that it's mean is at the origin
  void meanAdjustCameras(std::vector<bundler_parser::BundlerParser::Camera>& cameras);

  // estimate point cloud normals
  void estimateNormals(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const float search_radius);

  // estimate ground plane
  std::vector<double> estimateGroundPlane();

  // Visualization routine
  void visualize(bool show_cloud, bool show_cameras);

  // cameras
  std::vector<bundler_parser::BundlerParser::Camera> cameras;
  // point cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  // point normals
  pcl::PointCloud<pcl::Normal> normals;
  // ground plane
  std::vector<double> ground;

 private:

  // stores mean of all the points in the cloud
  std::vector<double> mean_point;
};

} // namespace pc_handler

#endif // OSM_PC_MAPPING_PC_HANDLER_H_
