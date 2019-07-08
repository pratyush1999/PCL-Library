#include <pcl/point_types.h>
#include <pcl/features/gasd.h>
#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include<pcl/visualization/pcl_plotter.h>
#include<pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
int main(int argc, char** argv)
{
 
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
      cloud->push_back(p);
      }
  }
    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  // Create the GASD estimation class, and pass the input dataset to it
  pcl::GASDEstimation<pcl::PointXYZ, pcl::GASDSignature512> gasd;
  gasd.setInputCloud (cloud);

  // Output datasets
  pcl::PointCloud<pcl::GASDSignature512> descriptor;

  // Compute the descriptor
  gasd.compute (descriptor);

  // Get the alignment transform
  // Eigen::Matrix4f::Ptr trans(new Eigen::Matrix4f) ;
//  gasd.getTransform (trans);

  // Unpack histogram bins
  for (size_t i = 0; i < size_t( descriptor[0].descriptorSize ()); ++i)
  {
    descriptor[0].histogram[i];
  }
   // visualize point cloud
     pcl::visualization::PCLVisualizer viewer_cloud("PCL Viewer");
     viewer_cloud.setBackgroundColor (0.0, 0.0, 0.5);

     viewer_cloud.addPointCloud<pcl::PointXYZ>(cloud);

     while (!viewer_cloud.wasStopped ())
     {
       viewer_cloud.spinOnce ();
     }

 // Plotter object.
 // pcl::visualization::PCLHistogramVisualizer viewer;
  // We need to set the size of the descriptor beforehand.
  // size_t i = 0;
  // viewer.addFeatureHistogram(*descriptor[0], 512);

  // viewer.spin();

  return 0;
}
