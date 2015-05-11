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

  // transform point cloud such that the normal is along z-axis
  void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_pointer,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cam_cloud_pointer);

  // Segmenting the point cloud
  void segmentPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                         const double threshold);

  // downproject the point cloud onto the ground plane to generate outline
  void generateOutline(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_pointer);

  // Visualization routine
  void visualize(bool show_cloud, bool show_cameras, bool show_normals);

  // cameras
  std::vector<bundler_parser::BundlerParser::Camera> cameras;
  // point cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  // point cloud outline
  pcl::PointCloud<pcl::PointXYZRGB> cloud_outline;
  // point normals
  pcl::PointCloud<pcl::Normal> normals;
  // ground plane
  std::vector<double> ground;
  // segmented point cloud
  pcl::PointCloud<pcl::PointXYZRGB> segmented_cloud;
  // point cloud of camera locations
  pcl::PointCloud<pcl::PointXYZ> cam_cloud;

 private:

  // stores mean of all the points in the cloud
  std::vector<double> mean_point;
};

} // namespace pc_handler

#endif // OSM_PC_MAPPING_PC_HANDLER_H_
