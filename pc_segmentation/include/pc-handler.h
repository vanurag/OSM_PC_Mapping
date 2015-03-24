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
      std::vector<bundler_parser::BundlerParser::Point3D> * points);

  // adjust the point cloud such that it's mean is at the origin
  void meanAdjustCloud();

  // Visualization routine
  void visualize();

  pcl::PointCloud<pcl::PointXYZRGB> cloud;

 private:

  // stores mean of all the points in the cloud
  std::vector<double> mean_point;
};

} // namespace pc_handler

#endif // OSM_PC_MAPPING_PC_HANDLER_H_
