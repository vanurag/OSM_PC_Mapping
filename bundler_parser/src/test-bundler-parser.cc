#include "common.h"
#include "bundler-parser.h"

using namespace bundler_parser;

int main(int num_arguments, char** arguments) {
  
  google::InitGoogleLogging(arguments[0]);
  google::ParseCommandLineFlags(&num_arguments, &arguments, true);
  google::InstallFailureSignalHandler();
  
  BundlerParser parser(FLAGS_file);
  
  CHECK_GT(FLAGS_index, -1) << "Provide a valid index (>= 0)";
  
  if (FLAGS_query == "camera") {
    BundlerParser::Camera camera = parser.getCamera(FLAGS_index);
    
    LOG(INFO) << "Camera Position:- " << camera.position.at(0) << ", "
                                      << camera.position.at(1) << ", "
                                      << camera.position.at(2); 
    
    LOG(INFO) << "Camera Pose:- ";
    for (int i = 0; i < 3; ++i) {
      LOG(INFO) << camera.pose.at(i).at(0) << ", "
                << camera.pose.at(i).at(1) << ", "
                << camera.pose.at(i).at(2); 
    }
    
    LOG(INFO) << "Intrinsics (f, k1, k2) = " << camera.focal_length 
              << ", " << camera.distortion_coeffs.first
              << ", " << camera.distortion_coeffs.second;
  }
  
  if (FLAGS_query == "3dpoint") {
    BundlerParser::Point3D point = parser.get3dPoint(FLAGS_index);
    
    LOG(INFO) << "3D Point coordinates:- " << point.position.at(0) << ", "
                                           << point.position.at(1) << ", "
                                           << point.position.at(2); 
    
    LOG(INFO) << "3D Point color:- " << static_cast<int>(point.color.at(0)) << ", "
                                     << static_cast<int>(point.color.at(1)) << ", "
                                     << static_cast<int>(point.color.at(2)); 
  }
  
  return 0;
}
