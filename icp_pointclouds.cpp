#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/colin/sandbox/icp_calibration/data/motoman_pointcloud_left_sampled.pcd", *cloud_out) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file /home/colin/sandbox/icp_calibration/data/motoman_pointcloud_left_sampled.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_out->width * cloud_out->height
            << " data points from /home/colin/sandbox/icp_calibration/data/motoman_pointcloud_left_sampled.pcd: "
            << std::endl;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/colin/sandbox/icp_calibration/data/kinect_base_sampled.pcd", *cloud_in) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file /home/colin/sandbox/icp_calibration/data/kinect_base.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_in->width * cloud_in->height
            << " data points from /home/colin/sandbox/icp_calibration/data/kinect_base.pcd: "
            << std::endl;



  // Fill in the CloudIn data
  // cloud_in->width    = 5;
  // cloud_in->height   = 1;
  // cloud_in->is_dense = false;
  // cloud_in->points.resize (cloud_in->width * cloud_in->height);
  // for (size_t i = 0; i < cloud_in->points.size (); ++i)
  // {
  //   cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
  //   cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
  //   cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  // }
  // std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
  //     << std::endl;
  // for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
  //     cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
  //     cloud_in->points[i].z << std::endl;
  // *cloud_out = *cloud_in;
  // std::cout << "size:" << cloud_out->points.size() << std::endl;
  // for (size_t i = 0; i < cloud_in->points.size (); ++i)
  //   cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  // std::cout << "Transformed " << cloud_in->points.size () << " data points:"
  //     << std::endl;
  // for (size_t i = 0; i < cloud_out->points.size (); ++i)
  //   std::cout << "    " << cloud_out->points[i].x << " " <<
  //     cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

 return (0);
}