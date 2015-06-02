#include "pc-handler.h"

namespace pc_handler {

void PcHandler::addToCloud(
    std::vector<bundler_parser::BundlerParser::Point3D>& points) {
  
  pcl::PointXYZRGB pc_point;
  int num_points = 0;
  
  for (std::vector<bundler_parser::BundlerParser::Point3D>::const_iterator it
           = points.begin(); it != points.end(); ++it) {
    
    // position
    pc_point.x = it->position(0);
    pc_point.y = it->position(1);
    pc_point.z = it->position(2);
    
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
    it->x -= mean_point.at(0);
    it->y -= mean_point.at(1);
    it->z -= mean_point.at(2);
  }
}


// adjust the camera centers such that it's mean is at the origin
void PcHandler::meanAdjustCameras(std::vector<bundler_parser::BundlerParser::Camera>& cameras) {
  // mean-centered camera centers
  for (std::vector<bundler_parser::BundlerParser::Camera>::iterator it = cameras.begin();
       it != cameras.end(); ++it) {
    it->center.at(0) -= mean_point.at(0);
    it->center.at(1) -= mean_point.at(1);
    it->center.at(2) -= mean_point.at(2);
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


// estimate ground plane
std::vector<double> PcHandler::estimateGroundPlane() {

  cv::Mat camera_positions;
  pcl::PointXYZ cam_point;
  //camera_positions.create(static_cast<int>(cameras.size()), 3, CV_64F);
  for (auto camera : cameras) {
    cv::Mat temp(camera.center);
    cv::Mat pos = temp.t();
    if (pos.at<double>(0, 0) != 0.0 || pos.at<double>(0, 1) != 0.0 || pos.at<double>(0, 2) != 0.0) {
      //LOG(INFO) << pos.at<double>(0, 0) << " " << pos.at<double>(0, 1) << " " << pos.at<double>(0, 2);
      camera_positions.push_back(pos);
      // cam cloud
      cam_point.x = pos.at<double>(0, 0);
      cam_point.y = pos.at<double>(0, 1);
      cam_point.z = pos.at<double>(0, 2);
      cam_cloud.push_back(cam_point);
    }
  }

  LOG(INFO) << "Solving PCA..." << camera_positions.size();
  cv::PCA ground_plane(camera_positions, cv::Mat(), 0);

  // Eigen values
  cv::Mat eigenvalues = ground_plane.eigenvalues;
  LOG(INFO) << "First three eigenvalues = " << eigenvalues.at<double>(0, 0) << ", "
            << eigenvalues.at<double>(0, 1) << ", " << eigenvalues.at<double>(0, 2);

  // Principal Axes
  cv::Mat pc = ground_plane.eigenvectors;
  for (int i = 0; i < 3; ++i) {
    LOG(INFO) << "PC " << i << ": [" << pc.at<double>(i, 0) << ", "
                                     << pc.at<double>(i, 1) << ", "
                                     << pc.at<double>(i, 2) << "]";
  }

  ground.push_back(pc.at<double>(2, 0));
  ground.push_back(pc.at<double>(2, 1));
  ground.push_back(pc.at<double>(2, 2));

  LOG(INFO) << "up-vector: " << ground.at(0) << " " << ground.at(1) << " " << ground.at(2);

  return ground;
}


// transform point cloud such that the normal is along z-axis
void PcHandler::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_pointer,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cam_cloud_pointer) {

  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
  Eigen::Vector3d original_normal;
  original_normal(0) = ground.at(0);
  original_normal(1) = ground.at(1);
  original_normal(2) = ground.at(2);
  Eigen::Vector3d desired_normal; // Z-axis
  desired_normal(0) = 0.0;
  desired_normal(1) = 0.0;
  desired_normal(2) = 1.0;

  double rot_angle =  acos(original_normal.dot(desired_normal));
  Eigen::Vector3d rot_axis = original_normal.cross(desired_normal);
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(rot_angle, rot_axis);
  transformation.block<3, 3>(0, 0) = rotation;

  // transform ground
  Eigen::Vector3d transformed_ground = rotation*original_normal;
  ground.at(0) = transformed_ground(0);
  ground.at(1) = transformed_ground(1);
  ground.at(2) = transformed_ground(2);
  LOG(INFO) << "up-vector after transformation: " << ground.at(0) << " " << ground.at(1) << " "
      << ground.at(2);

  // transform point cloud
  pcl::transformPointCloud(*cloud_pointer, *cloud_pointer, transformation);
  // transform camera locations
  pcl::transformPointCloud(*cam_cloud_pointer, *cam_cloud_pointer, transformation);
}


// Segmenting the point cloud
void PcHandler::segmentPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                                  const double threshold) {

  CHECK(cloud.size() == normals.size()) << "Point cloud and Normals size don't agree";
  pcl::PointCloud<pcl::Normal>::const_iterator itn = normals.begin();
  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator itp = cloud.begin(); itp != cloud.end();
       ++itp, ++itn) {
    Eigen::Vector3d point_normal_vector;
    point_normal_vector(0) = itn->normal_x;
    point_normal_vector(1) = itn->normal_y;
    point_normal_vector(2) = itn->normal_z;
    Eigen::Vector3d ground_normal_vector(ground.data());

    // segment out points with normals close to vertical to ground normal
    if (point_normal_vector.dot(ground_normal_vector) < threshold) {
      segmented_cloud.push_back(*itp);
    }
  }
}


// downproject the point cloud onto the ground plane to generate outline
void PcHandler::generateOutline(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_pointer) {
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);

  // input cloud
  proj.setInputCloud(cloud_pointer);

  // ground plane
  pcl::ModelCoefficients::Ptr model_coeffs_pointer(new pcl::ModelCoefficients ());
  model_coeffs_pointer->values.resize(4);
  model_coeffs_pointer->values[0] = ground.at(0);
  model_coeffs_pointer->values[1] = ground.at(1);
  model_coeffs_pointer->values[2] = ground.at(2);
  model_coeffs_pointer->values[3] = 0.0;
  proj.setModelCoefficients(model_coeffs_pointer);
  proj.filter(cloud_outline);
}


// Visualization routine
void PcHandler::visualize(bool show_cloud, bool show_cameras, bool show_normals) {
  pcl::visualization::PCLVisualizer pc_viewer("Point Cloud Viewer");
  pcl::visualization::PCLVisualizer cam_viewer("Camera Viewer");
  
  // Point cloud visualization
  int v1(0);
  pc_viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v1);
  pc_viewer.setBackgroundColor(0, 0, 0, v1);
  pc_viewer.addText ("Original Point Cloud", 10, 10, "v1 text", v1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pointer(&cloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_pointer);
  pc_viewer.addPointCloud<pcl::PointXYZRGB>(cloud_pointer, rgb, "point cloud", v1);
  pc_viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud", v1);
  // normals
  pcl::PointCloud<pcl::Normal>::Ptr normal_pointer(&normals);
  if (show_normals) {
    pc_viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>
        (cloud_pointer, normal_pointer, 10, 0.5, "point normals", v1);
  }
  // segmented point cloud
  int v2(0);
  pc_viewer.createViewPort (0.5, 0.0, 1.0, 0.5, v2);
  pc_viewer.setBackgroundColor (0, 0, 0, v2);
  pc_viewer.addText ("Segmented Point Cloud", 10, 10, "v2 text", v2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_cloud_pointer(&segmented_cloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> seg_rgb(seg_cloud_pointer);
  pc_viewer.addPointCloud<pcl::PointXYZRGB>(seg_cloud_pointer, seg_rgb, "segmented point cloud", v2);
  pc_viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud", v2);
  // point cloud outline downprojected onto the ground plane
  int v3(0);
  pc_viewer.createViewPort (0.0, 0.5, 1.0, 1.0, v3);
  pc_viewer.setBackgroundColor (0, 0, 0, v3);
  pc_viewer.addText ("Point Cloud Outline", 10, 10, "v3 text", v3);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outline_pointer(&cloud_outline);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> outline_rgb(outline_pointer);
  pc_viewer.addPointCloud<pcl::PointXYZRGB>(outline_pointer, outline_rgb,
                                            "point cloud outline", v3);
  pc_viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud outline", v3);
  // view facing downward
  pc_viewer.setCameraPosition(100.0*ground.at(0), 100.0*ground.at(1), 100.0*ground.at(2),
                              0.0, 0.0, 0.0);
  
  // camera positions visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_pointer(&cam_cloud);
  cam_viewer.setBackgroundColor(0, 0, 0);
  cam_viewer.addPointCloud<pcl::PointXYZ>(cam_cloud_pointer, "camera positions");
  cam_viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "camera positions");
  // Ground Plane
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back(ground.at(0));
  coeffs.values.push_back(ground.at(1));
  coeffs.values.push_back(ground.at(2));
  coeffs.values.push_back(0.0);
  cam_viewer.addPlane(coeffs, "plane");
  cam_viewer.addCoordinateSystem (1.0);
  // view facing downward
  cam_viewer.setCameraPosition(100.0*ground.at(0), 100.0*ground.at(1), 100.0*ground.at(2),
                               0.0, 0.0, 0.0);
    
  //pc_viewer.initCameraParameters();
  //cam_viewer.initCameraParameters();
  
  while ( (!pc_viewer.wasStopped()) && (!cam_viewer.wasStopped()) ) {
    pc_viewer.spinOnce (100);
    cam_viewer.spinOnce (100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

} // pc_handler

