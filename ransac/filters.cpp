#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>


typedef pcl::PointCloud< pcl::PointXY > twod;

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

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  auto begin = cloud->begin();
  auto end = cloud->end();

  twod::Ptr twodCloud (new twod);

  for(; begin != end; ++begin) {
    twodCloud->push_back(pcl::PointXY({begin->x, begin->y}));
    begin->z = 0;
  }

  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  auto viewer = simpleVis(cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr rs_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::RandomSample<pcl::PointXYZ> rs_filter(false);
  rs_filter.setInputCloud(cloud);
  rs_filter.setSample((cloud->width * cloud->height) / 2);

  rs_filter.filter(*rs_filtered_cloud);

  std::cout << "Now only "
          << rs_filtered_cloud->width * rs_filtered_cloud->height
          << " data points from test_pcd.pcd with the following fields: "
          << std::endl;


  // Implementation of further filter methods 
  // To reduce amount of point cloud data

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter(false);
  filter.setInputCloud(rs_filtered_cloud);
  filter.setRadiusSearch(2);
  filter.setMinNeighborsInRadius(50);
  filter.filter(*filtered_cloud);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter (false); // Initializing with true will allow us to extract the removed indices
  sorfilter.setInputCloud (filtered_cloud);
  sorfilter.setMeanK (8);
  sorfilter.setStddevMulThresh (1.0);
  sorfilter.filter (*filtered_cloud);

  rs_filter.setInputCloud(filtered_cloud);
  rs_filter.setSample((filtered_cloud->width * filtered_cloud->height) / 16);
  rs_filter.filter(*filtered_cloud);


  std::cout << "Now only "
          << filtered_cloud->width * filtered_cloud->height
          << " data points from test_pcd.pcd with the following fields: "
          << std::endl;


  auto viewer2 = simpleVis(filtered_cloud);

  pcl::io::savePCDFileASCII("filtered_output.pcd", *filtered_cloud);

  std::cout << "Saved everything to filtered_output.pcd" << std::endl;

  pcl::SampleConsensusModelLine <pcl::PointXYZ> sac_line (filtered_cloud);
  pcl::PointCloud <pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> inliers;
  Eigen::VectorXf coefficients;
  int iteration = 0;
  std::vector<int> sample_idx;
  while(inliers.size() == 0) {
    // sac_line.setDistanceThreshold(0.5);

    sac_line.getSamples(iteration, sample_idx);

    sac_line.computeModelCoefficients(sample_idx, coefficients);

    sac_line.selectWithinDistance(coefficients, 2, inliers);
    sac_line.projectPoints(inliers, coefficients, *final);
  }


  auto viewer3 = simpleVis(final);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    viewer2->spinOnce(100);
    viewer3->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
} 