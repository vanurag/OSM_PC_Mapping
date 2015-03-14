#include "bundler-parser.h"
#include "common.h"

namespace bundler_parser {

// returns Camera struct corresponding to the image_index-th image in the list.
// indexing from 0.
Camera BundlerParser::getCamera(int image_index) const {
  
  char temp[100];
  std::string::size_type next;
  
  // seek from beginning of the file
  seekToLine(file_handle, 2, false);
  file_handle.getline(temp, 90); // no. of images, no. of points
  int num_images = stoi(temp, &next);
  int num_points = stoi(temp.substr(next), &next);
  CHECK_GT(num_images, image_index) 
      << "specified image_index is higher than the total number of images!";
      
  seekToLine(file_handle, 5*image_index + 2, true); // 5 lines for each camera
  
  // Fill up Camera struct
  Camera camera;
  
  // focal_length, distortion parameters
  file_handle.getline(temp, 90);
  camera.focal_length = std::stod(temp, &next);
  double k1 = std::stod(temp.substr(next), &next); // radial distortion parameter along x
  double k2 = std::stod(temp.substr(next), &next); // radial distortion parameter along y
  camera.distortion_coeffs = std::make_pair(k1, k2);
  
  // Camera pose
  for (int i = 0; i < 3; ++row) {
    file_handle.getline(temp, 90);
    std::vector row;
    row.push_back(std::stod(temp, &next));
    row.push_back(std::stod(temp.substr(next), &next));
    row.push_back(std::stod(temp.substr(next), &next));
    
    camera.pose.push_back(row);
  }
  
  // Camera position
  file_handle.getline(temp, 90);
  camera.position.push_back(std::stod(temp, &next));
  camera.position.push_back(std::stod(temp.substr(next), &next));
  camera.position.push_back(std::stod(temp.substr(next), &next));
  
  return camera;
}


// returns index-th 3DPoint in the bundler file
// indexing from 0.
3DPoint BundlerParser::get3dPoint(int index) const {
  
  char temp[100];
  std::string::size_type next;
  
  // seek from beginning of the file
  seekToLine(file_handle, 2, false);
  file_handle.getline(temp, 90); // no. of images, no. of points
  int num_images = stoi(temp, &next);
  int num_points = stoi(temp.substr(next), &next);
  
  CHECK_GT(num_points, index) 
      << "specified index is higher than the total number of points!";
      
  seekToLine(file_handle, 5*num_images + 2, true); // 5 lines for each camera
  seekToLine(file_handle, 3*index + 1, true); // 3 lines for each point
  
  // Fill up 3DPoint struct
  3DPoint point;
  
  // 3D point position
  file_handle.getline(temp, 90);
  point.position.push_back(std::stod(temp, &next));
  point.position.push_back(std::stod(temp.substr(next), &next));
  point.position.push_back(std::stod(temp.substr(next), &next));
  
  // 3D point color
  file_handle.getline(temp, 90);
  point.color.push_back(static_cast<uint8_t>(std::stoi(temp, &next)));
  point.color.push_back(
      static_cast<uint8_t>(std::stoi(temp.substr(next), &next)) );
  point.color.push_back(
      static_cast<uint8_t>(std::stoi(temp.substr(next), &next)) );
  
  return point;
}

} // bundler_parser
