#include <iostream>
#include <random>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>

#include <string>
#include <cstdarg>

#include "sac_model_finite_line.hpp"

#include <pcl/kdtree/kdtree_flann.h>


using std::cout;
using std::endl;


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

inline std::string format(const char* fmt, ...){
    int size = 512;
    char* buffer = 0;
    buffer = new char[size];
    va_list vl;
    va_start(vl, fmt);
    int nsize = vsnprintf(buffer, size, fmt, vl);
    if(size<=nsize){ //fail delete buffer and try again
        delete[] buffer;
        buffer = 0;
        buffer = new char[nsize+1]; //+1 for /0
        nsize = vsnprintf(buffer, size, fmt, vl);
    }
    std::string ret(buffer);
    va_end(vl);
    delete[] buffer;
    return ret;
}


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // for the colors:
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1, 255);

  if(argc < 3) {
  	return (-1);
  }
  double threshold = atof(argv[1]);
  int end = atoi(argv[2]);
  cout << "threshold" << threshold << endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("filtered_output.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  int all_indicies = cloud->width * cloud->height;


  pcl::PointXYZ searchPoint;

  std::uniform_int_distribution<> rand_cloud(1, all_indicies);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  // pcl::PointCloud<pcl::PointXY>::Ptr two_d_cloud (new pcl::PointCloud<pcl::PointXY>);

  // for(auto p: *cloud) {
  //   two_d_cloud-> push_back(pcl::PointXY({p.x, p.y}));
  // }
  // cout << "Computed 2D cloud ...." << endl;

  int rand_idx = rand_cloud(gen);

  searchPoint = (*cloud)[rand_idx];


  kdtree.setInputCloud(cloud);

  int K = 10;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);



  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }

  pcl::SampleConsensusModelFiniteLine<pcl::PointXYZ>::Ptr line_model (
    new pcl::SampleConsensusModelFiniteLine<pcl::PointXYZ> (cloud));

  line_model->kdtree = kdtree;
  
  std::vector<pcl::PointCloud <pcl::PointXYZ>::Ptr > finals;


  Eigen::VectorXf model_coefficients;
  line_model->computeModelCoefficients({rand_idx, pointIdxNKNSearch[2]}, model_coefficients);
  cout << "Coeffs " << model_coefficients << endl;
  
  auto viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "reference");

  boost::shared_ptr< std::vector<int> > inliers (new std::vector<int>);

  auto filter = pcl::ExtractIndices<pcl::PointXYZ>();
  filter.setNegative(true);

  // for(int i = 0; i <= end; i++) {
  //   std::string pcname = format("ransacd %i", i);
  //   pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (line_model);
  //   ransac.setDistanceThreshold(threshold);
  //   ransac.computeModel();
  //   ransac.getInliers(*inliers);
  //   finals.push_back(pcl::PointCloud <pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));

  //   pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *finals.back());
  //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (finals.back(), dis(gen), dis(gen) * i / end, dis(gen) * i / end);
  //   viewer->addPointCloud<pcl::PointXYZ> (finals.back(), single_color, pcname);
  //   // pcl_ind.indices(inliers);
  //   filter.setIndices(inliers);
  //   filter.filterDirectly(cloud);
  // }

  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
} 