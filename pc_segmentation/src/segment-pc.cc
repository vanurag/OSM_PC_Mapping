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
  for (int i = 0; i < parser.num_points; ++i) {
    point_indices.push_back(i);
  }
  std::vector<BundlerParser::Point3D> points = parser.get3dPoints(point_indices);

  pcl::PointXYZRGB pc_point;
  std::vector<double> mean_point (3, 0.0);
  for (auto point : points) {
    
    // position
    pc_point.x = point.position.at(0);
    pc_point.y = point.position.at(1);
    pc_point.z = point.position.at(2);
    
    // color
    pc_point.r = point.color.at(0);
    pc_point.g = point.color.at(1);
    pc_point.b = point.color.at(2);
    
    // cloud
    pc_handle.cloud.push_back(pc_point);
    
    // update mean
    mean_point.at(0) += pc_point.x;
    mean_point.at(1) += pc_point.y;
    mean_point.at(2) += pc_point.z;
  }
  
  // mean
  mean_point.at(0) /= parser.num_points;
  mean_point.at(1) /= parser.num_points;
  mean_point.at(2) /= parser.num_points;
  
  LOG(INFO) << "Mean position: " << mean_point.at(0) << ", "
                                 << mean_point.at(1) << ", "
                                 << mean_point.at(2);
  
  // mean-centered point cloud
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = pc_handle.cloud.begin();
       it != pc_handle.cloud.end(); ++it) {
    (*it).x -= mean_point.at(0);
    (*it).y -= mean_point.at(1);
    (*it).z -= mean_point.at(2);
  }
  
  // visualize
  //pcl::visualization::CloudViewer cloud_viewer("point cloud");
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pointer(&pc_handle.cloud);
  //cloud_viewer.showCloud(cloud_pointer, "point cloud");

  //while (! cloud_viewer.wasStopped()) {
  //}
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer 
      (new pcl::visualization::PCLVisualizer ("point cloud"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pointer(&pc_handle.cloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_pointer);
  
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_pointer, rgb, "aachen");
  
  //pc_point.x = mean_point.at(0);
  //pc_point.y = mean_point.at(1);
  //pc_point.z = mean_point.at(2);
  //viewer->addSphere (pc_point, 10.0, 0.5, 0.5, 0.0, "sphere");
  
  
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "aachen");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  
  while (! viewer->wasStopped())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  

  return 0;
}
