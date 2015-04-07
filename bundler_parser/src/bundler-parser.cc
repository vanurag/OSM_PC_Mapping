#include "bundler-parser.h"
#include "common.h"
#include "utility-functions.h"

namespace bundler_parser {
  
// Constructor
BundlerParser::BundlerParser(const std::string filename) {
  file_handle.open(filename, std::ifstream::in);
  
  char temp[100];
  std::string temp_string;
  std::string::size_type next;
  
  // read meta-data
  seekToLine(file_handle, 2, false);
  file_handle.getline(temp, 90); // no. of images, no. of points
  temp_string = std::string(temp);
  num_images = std::stoi(temp_string, &next);
  num_points = std::stoi(temp_string.substr(next), &next);
  LOG(INFO) << "No. Images: " << num_images;
  LOG(INFO) << "No. Points: " << num_points;
  LOG(INFO) << "--------------";
};


// Destructor
BundlerParser::~BundlerParser() {
  file_handle.close();
};

// returns set of Camera structs corresponding to those in image_indices list.
// indexing from 0.
std::vector<BundlerParser::Camera> BundlerParser::getCameras(
    std::vector<int> image_indices) {
  
  char temp[100];
  std::string temp_string;
  std::string::size_type next;
  
  // Sort indices
  std::sort(image_indices.begin(), image_indices.end());
  
  // Check if valid indices
  CHECK_GT(image_indices.at(0), -1) 
      << "Provide a valid index (>= 0)";
  CHECK_GT(num_images, image_indices.at(image_indices.size() - 1)) 
      << "not all specified image_indices are less than the total number of images!";
      
  // seek from beginning of the file
  seekToLine(file_handle, 3, false);
  
  // Vector of camers queried
  std::vector<BundlerParser::Camera> cameras;
  int prev_index = -1;
  
  for (auto index : image_indices) {
    
    // Seek to the next camera in the query list. 5 lines for each camera
    seekToLine(file_handle, 5*(index - prev_index - 1) + 1, true); 
    prev_index = index;
  
    // Fill up Camera struct
    Camera camera;
    
    // focal_length, distortion parameters
    file_handle.getline(temp, 90);
    temp_string = std::string(temp);
    camera.focal_length = std::stod(temp_string, &next);
    // radial distortion parameter along x
    temp_string = temp_string.substr(next);
    double k1 = std::stod(temp_string, &next); 
    // radial distortion parameter along y
    temp_string = temp_string.substr(next);
    double k2 = std::stod(temp_string, &next); 
    camera.distortion_coeffs = std::make_pair(k1, k2);
    
    // Camera pose
    for (int i = 0; i < 3; ++i) {
      file_handle.getline(temp, 90);
      temp_string = std::string(temp);
      camera.pose(i, 0) = std::stod(temp_string, &next);
      temp_string = temp_string.substr(next);
      camera.pose(i, 1) = std::stod(temp_string, &next);
      temp_string = temp_string.substr(next);
      camera.pose(i, 2) = std::stod(temp_string, &next);
    }
    
    // Camera translation
    file_handle.getline(temp, 90);
    temp_string = std::string(temp);
    camera.translation(0) = std::stod(temp_string, &next);
    temp_string = temp_string.substr(next);
    camera.translation(1) = std::stod(temp_string, &next);
    temp_string = temp_string.substr(next);
    camera.translation(2) = std::stod(temp_string, &next);

    // Camera center
    Eigen::Vector3d camera_center = -1.0 * camera.pose.transpose() * camera.translation;
    camera.center.push_back(camera_center(0));
    camera.center.push_back(camera_center(1));
    camera.center.push_back(camera_center(2));
    
    cameras.push_back(camera);
  }
    
  return cameras;
}


// returns set of Point3D structs corresponding to those in point_indices list.
// indexing from 0.
std::vector<BundlerParser::Point3D> BundlerParser::get3dPoints(
    std::vector<int> point_indices) {
  
  char temp[10000];
  std::string temp_string;
  std::string::size_type next;
  
  // Sort indices
  std::sort(point_indices.begin(), point_indices.end());
  
  // Check if valid indices
  CHECK_GT(point_indices.at(0), -1) 
      << "Provide a valid index (>= 0)";
  CHECK_GT(num_points, point_indices.at(point_indices.size() - 1)) 
      << "Not all specified indices are lower than the total number of points!";
      
  // seek from beginning of the file
  seekToLine(file_handle, 3, false);
     
  // skip camera data
  seekToLine(file_handle, 5*num_images + 1, true); // 5 lines for each camera
  
  // Vector of 3d points queried
  std::vector<BundlerParser::Point3D> points;
  int prev_index = -1;
  
  for (auto index : point_indices) {
    
    // seek to the next point index. 3 lines for each point
    seekToLine(file_handle, 3*(index - prev_index - 1) + 1, true); 
    prev_index = index;
    
    // Fill up Point3D struct
    Point3D point;
    
    // 3D point position
    file_handle.getline(temp, 90);
    temp_string = std::string(temp);
    point.position(0) = std::stod(temp_string, &next);
    temp_string = temp_string.substr(next);
    point.position(1) = std::stod(temp_string, &next);
    temp_string = temp_string.substr(next);
    point.position(2) = std::stod(temp_string, &next);
    
    // 3D point color
    file_handle.getline(temp, 90);
    temp_string = std::string(temp);
    point.color.push_back(static_cast<uint8_t>(std::stoi(temp_string, &next)));
    temp_string = temp_string.substr(next);
    point.color.push_back(
        static_cast<uint8_t>(std::stoi(temp_string, &next)) );
    temp_string = temp_string.substr(next);
    point.color.push_back(
        static_cast<uint8_t>(std::stoi(temp_string, &next)) );
    
    // skip parsing view list, for now
    file_handle.getline(temp, 9000);
    
    points.push_back(point);
  }
  
  return points;
}


// Print Camera struct
void BundlerParser::printCamera(const BundlerParser::Camera& camera) const {
  LOG(INFO) << "Camera Position:- " << camera.center.at(0) << ", "
                                      << camera.center.at(1) << ", "
                                      << camera.center.at(2);

  LOG(INFO) << "Camera Translation:- " << camera.translation(0) << ", "
                                        << camera.translation(1) << ", "
                                        << camera.translation(2);
    
  LOG(INFO) << "Camera Pose:- ";
  for (int i = 0; i < 3; ++i) {
    LOG(INFO) << camera.pose(i, 0) << ", "
              << camera.pose(i, 1) << ", "
              << camera.pose(i, 2);
  }
  
  LOG(INFO) << "Intrinsics (f, k1, k2) = " << camera.focal_length 
            << ", " << camera.distortion_coeffs.first
            << ", " << camera.distortion_coeffs.second;
}


// Print Point3D struct
void BundlerParser::print3DPoint(const BundlerParser::Point3D& point) const {
  LOG(INFO) << "3D Point coordinates:- " << point.position(0) << ", "
                                       << point.position(1) << ", "
                                       << point.position(2);

  LOG(INFO) << "3D Point color:- " << static_cast<int>(point.color.at(0)) << ", "
                                   << static_cast<int>(point.color.at(1)) << ", "
                                   << static_cast<int>(point.color.at(2)); 
}

} // bundler_parser
