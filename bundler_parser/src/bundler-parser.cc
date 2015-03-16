#include "bundler-parser.h"
#include "common.h"

namespace bundler_parser {
  
// Constructor
BundlerParser::BundlerParser(const std::string filename) {
  file_handle.open(filename, std::ifstream::in);
};


// Destructor
BundlerParser::~BundlerParser() {
  file_handle.close();
};

// returns Camera struct corresponding to the image_index-th image in the list.
// indexing from 0.
BundlerParser::Camera BundlerParser::getCamera(int image_index) {
  
  char temp[100];
  std::string temp_string;
  std::string::size_type next;
  
  // seek from beginning of the file
  seekToLine(file_handle, 2, false);
  file_handle.getline(temp, 90); // no. of images, no. of points
  temp_string = std::string(temp);
  int num_images = std::stoi(temp_string, &next);
  int num_points = std::stoi(temp_string.substr(next), &next);
  LOG(INFO) << "No. Images: " << num_images;
  LOG(INFO) << "No. Points: " << num_points;
  CHECK_GT(num_images, image_index) 
      << "specified image_index is higher than the total number of images!";
      
  seekToLine(file_handle, 5*image_index + 1, true); // 5 lines for each camera
  
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
    std::vector<double> row;
    row.push_back(std::stod(temp_string, &next));
    temp_string = temp_string.substr(next);
    row.push_back(std::stod(temp_string, &next));
    temp_string = temp_string.substr(next);
    row.push_back(std::stod(temp_string, &next));
    
    camera.pose.push_back(row);
  }
  
  // Camera position
  file_handle.getline(temp, 90);
  temp_string = std::string(temp);
  camera.position.push_back(std::stod(temp_string, &next));
  temp_string = temp_string.substr(next);
  camera.position.push_back(std::stod(temp_string, &next));
  temp_string = temp_string.substr(next);
  camera.position.push_back(std::stod(temp_string, &next));
  
  return camera;
}


// returns index-th 3DPoint in the bundler file
// indexing from 0.
BundlerParser::Point3D BundlerParser::get3dPoint(int index) {
  
  char temp[100];
  std::string temp_string;
  std::string::size_type next;
  
  // seek from beginning of the file
  seekToLine(file_handle, 2, false);
  file_handle.getline(temp, 90); // no. of images, no. of points
  temp_string = std::string(temp);
  int num_images = std::stoi(temp_string, &next);
  int num_points = std::stoi(temp_string.substr(next), &next);
  LOG(INFO) << "No. Images: " << num_images;
  LOG(INFO) << "No. Points: " << num_points;
  
  CHECK_GT(num_points, index) 
      << "specified index is higher than the total number of points!";
      
  seekToLine(file_handle, 5*num_images + 1, true); // 5 lines for each camera
  seekToLine(file_handle, 3*index + 1, true); // 3 lines for each point
  
  // Fill up Point3D struct
  Point3D point;
  
  // 3D point position
  file_handle.getline(temp, 90);
  temp_string = std::string(temp);
  point.position.push_back(std::stod(temp_string, &next));
  temp_string = temp_string.substr(next);
  point.position.push_back(std::stod(temp_string, &next));
  temp_string = temp_string.substr(next);
  point.position.push_back(std::stod(temp_string, &next));
  
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
  
  return point;
}

} // bundler_parser
