#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

#include <tf/transform_listener.h>
#include "ros/ros.h"

#include <Eigen/Core>

using namespace pcl;

tf::TransformListener * tf_l;

int
 main (int argc, char** argv)
{
  if (argc < 4) {
    std::cerr << "Usage: " << argv[1] << " <source_pcd> <target_pcd> <tf_camera_frame> ";
    return(-1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  std::string source_pcd = argv[1];
  std::string target_pcd = argv[2];

  std::string file_in = "/home/pracsys/Repos/icp_calibration/data/" + source_pcd;
  std::string file_target = "/home/pracsys/Repos/icp_calibration/data/" + target_pcd;


  ros::init(argc, argv, "icp_calibration");
  ros::NodeHandle main_node_handle;
  tf_l = new tf::TransformListener;
  tf::StampedTransform camera_tf;


  // Load input pointcloud from Kinect camera
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_in, *cloud_in) == -1) 
  {
    PCL_ERROR ("Couldn't read file  \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_in->width * cloud_in->height
            << " data points from " + file_in
            << std::endl;

  // Load reference ground truth pointcloud
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_target, *cloud_out) == -1)
  {
    PCL_ERROR ("Couldn't read file  \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_out->width * cloud_out->height
            << " data points from " + file_target
            << std::endl;

  // Process input pointcloud to mask past certain depth (2.0m)
  // for (size_t i = 0; i < cloud_in->points.size(); i++) {
  //   if (cloud_in->points[i].z > 2.0) {
  //     cloud_in->erase(cloud_in->begin()+(i-1));
  //   }
  // }

  std::cout << "finished editing pointcloud" << std::endl;

  // Get TF from camera frame to localize with respect to
  std::string from_frame = argv[3];
  std::string to_frame   = "map";

  try {
      tf_l->waitForTransform(from_frame, to_frame, ros::Time(0), ros::Duration(5));
      tf_l->lookupTransform( from_frame, to_frame, 
                                          ros::Time(0), camera_tf);
  }
  catch (tf::TransformException except) {
      std::cerr << "Found no TF from "
                  << from_frame.c_str() << " to " << to_frame.c_str()
                  << " -> " << except.what();
  }

  // Transform incoming point cloud to a reasonable guess of camera pose 
  const Eigen::Vector3f translate (camera_tf.getOrigin().getX(), 
                                    camera_tf.getOrigin().getY(), 
                                    camera_tf.getOrigin().getZ());
  // const Eigen::Vector3f translate (-0.779, 0.089, -0.120);
  std::cout << "translation: " << translate << std::endl;

  // Format: WXYZ
  // Input from tf_echo (default): XYZW
  const Eigen::Quaternionf rotate ( camera_tf.getRotation().getW(),
                                    camera_tf.getRotation()[0], 
                                    camera_tf.getRotation()[1], 
                                    camera_tf.getRotation()[2]
                                    );

  // const Eigen::Quaternionf rotate (0.965, -0.054, 0.255, 0.015);  

  std::cout << "rotation: " << rotate.w() << " " <<rotate.x() << " " << rotate.y() << " " << rotate.z() << " " << std::endl;

  pcl::transformPointCloud(*cloud_out, *cloud_out, translate, rotate);

  pcl::PointCloud<pcl::PointXYZRGB> in_color;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_color_ptr (&in_color);

  // Visualize BEFORE
  in_color.points.resize(cloud_in->size());
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    in_color.points[i].x = cloud_in->points[i].x;
    in_color.points[i].y = cloud_in->points[i].y;
    in_color.points[i].z = cloud_in->points[i].z;
    in_color.points[i].r = 0;
    in_color.points[i].g = 100;
    in_color.points[i].b = 0;
     
  }

  visualization::PCLVisualizer vis_before;
  vis_before.addPointCloud (in_color_ptr);
  vis_before.addPointCloud (cloud_out, "reference_cloud");
  vis_before.spin ();

  // Apply ICP
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  // Set ICP parameters
  icp.setMaxCorrespondenceDistance(0.05);
  // std::cout << "current trans-eps: " << icp.getTransformationEpsilon() << std::endl;
  // icp.setTransformationEpsilon(0.5);
  // std::cout << "current trans-eps: " << icp.getTransformationEpsilon() << std::endl;

  icp.setInputSource(cloud_in);
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

  // Visualize AFTER
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

  visualization::PCLVisualizer vis_sampled;
  vis_sampled.addPointCloud (Final_color_ptr);
  vis_sampled.addPointCloud (cloud_out, "reference_cloud");
  vis_sampled.spin ();
  
  // pcl::io::savePCDFileASCII ("/home/colin/sandbox/icp_calibration/data/kinect_base_sampled_transformed.pcd", *cloud_in);

 return (0);
}