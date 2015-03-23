#ifndef OSM_PC_MAPPING_BUNDLER_PARSER_H_
#define OSM_PC_MAPPING_BUNDLER_PARSER_H_

#include "common.h"

namespace bundler_parser {

class BundlerParser {
 public:
  BundlerParser(const std::string filename);
  virtual ~BundlerParser();

  struct Camera {
    std::vector<double> position;
    std::vector<std::vector<double>> pose;
    double focal_length;
    std::pair<double, double> distortion_coeffs;
  };

  struct Point3D {
    std::vector<double> position;
    std::vector<uint8_t> color;
    int num_cameras;                        // no. of cameras with this 3d point in their FOV

    // list of camera indices with this 3D point in their FOV and the position at which it appears in the image
    std::vector<std::pair<int, std::pair<int, int>>> view_list;
  };

  // returns set of Camera structs corresponding to those in image_indices list.
  // indexing from 0.
  std::vector<BundlerParser::Camera> getCameras(std::vector<int> image_indices);

  // returns set of Point3D structs corresponding to those in point_indices list.
  // indexing from 0.
  std::vector<BundlerParser::Point3D> get3dPoints(std::vector<int> point_indices);

  // Print camera struct
  void printCamera(const Camera& camera) const;

  // Print Point3D struct
  void print3DPoint(const Point3D& point) const;

  // meta-data
  int num_images;
  int num_points;

 private:

  std::fstream file_handle;
};

} // namespace bundler_parser

#endif // OSM_PC_MAPPING_BUNDLER_PARSER_H_