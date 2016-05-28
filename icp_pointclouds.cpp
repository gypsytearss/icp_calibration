#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>

using namespace pcl;

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

  // Transform incoming point cloud to a reasonable guess of camera pose 
  const Eigen::Vector3f translate (0.307, 0.680, 0.941);

  // Format: WXYZ
  // Input from tf_echo (default): XYZW
  const Eigen::Quaternionf rotate (-0.414, -0.567, 0.661, -0.263);

  pcl::transformPointCloud(*cloud_in, *cloud_in, translate, rotate);

  // Apply ICP
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final_ptr (&Final);
  icp.align(Final);

  // Print out 
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;


  Eigen::Matrix4f final_transform = icp.getFinalTransformation();

  Eigen::Matrix3f trans_rot;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      trans_rot(i,j) = final_transform(i,j);

  Eigen::Vector3f trans_trans (final_transform(0,3),
                                final_transform(1,3),
                                final_transform(2,3));
  Eigen::Quaternionf trans_q (trans_rot);
  std::cout << "Translation: " << trans_trans << std::endl;
  std::cout << "Rotation: " << trans_q.w() << " "
                            << trans_q.x() << " "
                            << trans_q.y() << " "
                            << trans_q.z() << " "
                            << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB> Final_color;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final_color_ptr (&Final_color);

  Final_color.points.resize(Final.size());
  for (size_t i = 0; i < Final.points.size(); i++) {
    Final_color.points[i].x = Final.points[i].x;
    Final_color.points[i].y = Final.points[i].y;
    Final_color.points[i].z = Final.points[i].z;
    Final_color.points[i].r = 0;
    Final_color.points[i].g = 100;
    Final_color.points[i].b = 0;
     
  }

  // Visualize
  visualization::PCLVisualizer vis_sampled;
  vis_sampled.addPointCloud (Final_color_ptr);
  vis_sampled.addPointCloud (cloud_out, "reference_cloud");
  vis_sampled.spin ();
  
  // pcl::io::savePCDFileASCII ("/home/colin/sandbox/icp_calibration/data/kinect_base_sampled_transformed.pcd", *cloud_in);

 return (0);
}