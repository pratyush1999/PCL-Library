#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include<pcl/visualization/pcl_plotter.h>
#include<pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv){
 typedef pcl::PointXYZ InputPointType;
  pcl::PointCloud<InputPointType>::Ptr cloud (new pcl::PointCloud<InputPointType>);
  if(argc > 1)
  {
    // Read the point cloud from the first command line argument
    std::string fileName = argv[1];
    std::cout << "Reading " << fileName << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file");
      return (-1);
    }
  }
  else
  {
    // Create a point cloud
    for(unsigned int i = 0; i < 1e3; ++i)
      {
      InputPointType p; 
      // Random coordinate
      p.x = drand48(); p.y = drand48(); p.z = drand48();

      //std::cout << "p: " << p << std::endl;
     // cout<<p.x<<" "<<p.y<<" "<<p.z<<"\n";
      cloud->push_back(p);
      }
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;
   //for(auto x:cloud->points)
   // cout<<x<<" ";
  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  pcl::search::KdTree<InputPointType>::Ptr tree_for_normal_estimation (new pcl::search::KdTree<InputPointType>);
  normalEstimation.setSearchMethod (tree_for_normal_estimation);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.1);

  std::cout << "Computing normals..." << std::endl;
  normalEstimation.compute (*cloudWithNormals);

  // Create the VFH estimation class, and pass the input dataset+normals to it
   std::cout << "Computing features..." << std::endl;
  
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (cloudWithNormals);
  // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<InputPointType>::Ptr tree (new pcl::search::KdTree<InputPointType> ());
  vfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the features
  vfh.compute (*vfhs);

   std::cout << "output points.size (): " << vfhs->points.size () << std::endl;
  // vfhs->points.size () should have the same size as the input cloud->points.size ()*

      pcl::visualization::PCLVisualizer viewer_cloud("PCL Viewer");
     viewer_cloud.setBackgroundColor (0.0, 0.0, 0.5);

     viewer_cloud.addPointCloud<pcl::PointXYZ>(cloud);

     while (!viewer_cloud.wasStopped ())
     {
       viewer_cloud.spinOnce ();
     }
    
// Plotter object.
    //   for(auto x:vfhs->points)
    // cout<<x<<" ";
  pcl::visualization::PCLHistogramVisualizer viewer;
  // We need to set the size of the descriptor beforehand.
  viewer.addFeatureHistogram(*vfhs, 308);

  viewer.spin();


  return 0;

}
