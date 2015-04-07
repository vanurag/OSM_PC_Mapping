#ifndef PC_SEGMENTATION_COMMON_H_
#define PC_SEGMENTATION_COMMON_H_

#include <bundler-parser.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

// STL headers.
#include <cmath>
#include <ctype.h>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

// Boost
#include <boost/multi_array.hpp>
#include <boost/thread/thread.hpp>

// GFlags, GLog and GTest headers.
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

// OpenCV
#include <opencv2/core/core.hpp>

#endif // PC_SEGMENTATION_COMMON_H_
