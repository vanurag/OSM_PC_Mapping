#include "pc-handler.h"
#include "common.h"

namespace pc_handler {

void PcHandler::addToCloud(
    std::vector<bundler_parser::BundlerParser::Point3D> * points) {
  
  pcl::PointXYZRGB pc_point;
  int num_points = 0;
  
  for (std::vector<bundler_parser::BundlerParser::Point3D>::const_iterator it
           = (*points).begin(); it != (*points).end(); ++it) {
    
    // position
    pc_point.x = it->position.at(0);
    pc_point.y = it->position.at(1);
    pc_point.z = it->position.at(2);
    
    // color
    pc_point.r = it->color.at(0);
    pc_point.g = it->color.at(1);
    pc_point.b = it->color.at(2);
    
    // register to cloud
    cloud.push_back(pc_point);
    
    // update mean
    mean_point.at(0) += pc_point.x;
    mean_point.at(1) += pc_point.y;
    mean_point.at(2) += pc_point.z;
    
    ++num_points;
  }
  
  // mean
  mean_point.at(0) /= num_points;
  mean_point.at(1) /= num_points;
  mean_point.at(2) /= num_points;
  
  LOG(INFO) << "Mean position: " << mean_point.at(0) << ", "
                                 << mean_point.at(1) << ", "
                                 << mean_point.at(2);
}


// adjust the point cloud such that it's mean is at the origin
void PcHandler::meanAdjustCloud() {
  // mean-centered point cloud
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud.begin();
       it != cloud.end(); ++it) {
    (*it).x -= mean_point.at(0);
    (*it).y -= mean_point.at(1);
    (*it).z -= mean_point.at(2);
  }
}


// Visualization routine
void PcHandler::visualize() {
  //pcl::visualization::CloudViewer cloud_viewer("point cloud");
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pointer(&pc_handle.cloud);
  //cloud_viewer.showCloud(cloud_pointer, "point cloud");

  //while (! cloud_viewer.wasStopped()) {
  //}
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer 
      (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pointer(&cloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_pointer);
  
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_pointer, rgb, "point cloud");
  
  //pcl::PointXYZRGB pc_point;
  //pc_point.x = pc_handle.mean_point.at(0);
  //pc_point.y = pc_handle.mean_point.at(1);
  //pc_point.z = pc_handle.mean_point.at(2);
  //viewer->addSphere (pc_point, 10.0, 0.5, 0.5, 0.0, "sphere");
  
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  
  while (! viewer->wasStopped())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

} // pc_handler

