#include "pc-handler.h"
#include "common.h"

namespace pc_handler {

void PcHandler::addToCloud(
    std::vector<bundler_parser::BundlerParser::Point3D>& points) {
  
  pcl::PointXYZRGB pc_point;
  int num_points = 0;
  
  for (std::vector<bundler_parser::BundlerParser::Point3D>::const_iterator it
           = points.begin(); it != points.end(); ++it) {
    
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
void PcHandler::meanAdjustCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
  // mean-centered point cloud
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud.begin();
       it != cloud.end(); ++it) {
    (*it).x -= mean_point.at(0);
    (*it).y -= mean_point.at(1);
    (*it).z -= mean_point.at(2);
  }
}


// estimate point cloud normals
void PcHandler::estimateNormals(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const float search_radius) {

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset.
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);

  // Use all neighbors in search_radius
  ne.setRadiusSearch(search_radius);

  // Compute the features
  ne.compute(normals);
}


// Visualization routine
void PcHandler::visualize(bool show_cloud, bool show_cameras) {
  pcl::visualization::PCLVisualizer pc_viewer("Point Cloud Viewer");
  pcl::visualization::PCLVisualizer cam_viewer("Camera Viewer");
  pc_viewer.setBackgroundColor(0, 0, 0);
  cam_viewer.setBackgroundColor(0, 0, 0);
  
  // Point cloud visualization
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pointer(&cloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_pointer);
  pc_viewer.addPointCloud<pcl::PointXYZRGB>(cloud_pointer, rgb, "point cloud");
  pc_viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud");
  // normals
  estimateNormals(cloud_pointer, 0.4);
  pcl::PointCloud<pcl::Normal>::Ptr normal_pointer(&normals);
  pc_viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>
      (cloud_pointer, normal_pointer, 10, 0.5, "point normals");
  pc_viewer.addCoordinateSystem(1.0);
  
  // camera positions visualization
  pcl::PointCloud<pcl::PointXYZ> cam_cloud;
  pcl::PointXYZ cam_point;
  for (std::vector<bundler_parser::BundlerParser::Camera>::iterator it = 
           cameras.begin();
       it != cameras.end(); ++it) {
    // position
    cam_point.x = it->position.at(0);
    cam_point.y = it->position.at(1);
    cam_point.z = it->position.at(2);
    
    cam_cloud.push_back(cam_point);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_pointer(&cam_cloud);
  cam_viewer.addPointCloud<pcl::PointXYZ>(cam_cloud_pointer, "camera positions");
  cam_viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "camera positions");
  cam_viewer.addCoordinateSystem (1.0);
    
  pc_viewer.initCameraParameters();
  cam_viewer.initCameraParameters();
  
  while ( (!pc_viewer.wasStopped()) && (!cam_viewer.wasStopped()) ) {
    pc_viewer.spinOnce (100);
    cam_viewer.spinOnce (100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

} // pc_handler

