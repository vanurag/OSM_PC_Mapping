#ifndef COMMON_H_
#define COMMON_H_

#include <flags.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>

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

#endif // COMMON_H_
