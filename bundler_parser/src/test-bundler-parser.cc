#include "common.h"
#include "bundler-parser.h"

using namespace bundler_parser;

int main(int num_arguments, char** arguments) {
  
  BundlerParser parser(FLAGS_file);
  
  CHECK_GT(FLAGS_index, -1) << "Provide a valid index (>= 0)";
  
  if (FLAGS_task == "camera") {
    BundlerParser::Camera camera = parser.getCamera(FLAGS_index);
    
    LOG(INFO) << "Camera Position:- " << camera.position; 
    
    LOG(INFO) << "Camera Pose:- ";
    for (int i = 0; i < 3; ++i) {
      LOG(INFO) << camera.pose.at(i);
    }
    
    LOG(INFO) << "Intrinsics (f, k1, k2) = " << camera.focal_length 
              << " " << camera.distortion_coeffs.first
              << " " << camera.distortion_coeffs.second;
  }
  
  if (FLAGS_task == "3dpoint") {
    BundlerParser::3DPoint point = parser.get3dPoint(FLAGS_index);
    
    LOG(INFO) << "3D Point coordinates:- " << point.position;
    
    LOG(INFO) << "3D Point color:- " << point.color;
  }
  
  return 0;
}
