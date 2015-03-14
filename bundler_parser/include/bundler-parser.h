#ifndef OSM_PC_MAPPING_BUNDLER_PARSER_H_
#define OSM_PC_MAPPING_BUNDLER_PARSER_H_

#include common.h

namespace bundler_parser {

class BundlerParser {
 public:
  BundlerParser(const std::string filename) {
    file_handle.open (filename, std::ifstream::in);
  };
  virtual ~BundlerParser() {
    file_handle.close()
  };

  struct Camera {
    std::vector<double> position;
    std::vector<std::vector<double>> pose;
    double focal_length;
    std::pair<double, double> distortion_coeffs;
  };

  struct 3DPoint {
    std::vector<double> position;
    std::vector<uint8_t> color;
    int num_cameras;                        // no. of cameras with this 3d point in their FOV

    // list of camera indices with this 3D point in their FOV and the position at which it appears in the image
    std::vector<std::pair<int, std::pair<x, y>>> view_list;
  };

  // returns Camera struct corresponding to the image_index-th image in the list.
  // indexing from 0.
  Camera getCamera(int image_index) const;

  // returns index-th 3DPoint in the bundler file
  // indexing from 0.
  3DPoint get3dPoint(int index) const;

 private:

  std::ifstream& file_handle;
};

} // namespace bundler_parser

#endif // OSM_PC_MAPPING_BUNDLER_PARSER_H_