#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
int
main (int argc, char** argv){
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

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);
  // visualize point cloud
     pcl::visualization::PCLVisualizer viewer_cloud("PCL Viewer");
     viewer_cloud.setBackgroundColor (0.0, 0.0, 0.5);

     viewer_cloud.addPointCloud<pcl::PointXYZ>(cloud);

     while (!viewer_cloud.wasStopped ())
     {
       viewer_cloud.spinOnce ();
     }
    
    // visualize normals
     pcl::visualization::PCLVisualizer viewer("PCL Viewer");
     viewer.setBackgroundColor (0.0, 0.0, 0.5);
     
     viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);

     while (!viewer.wasStopped ())
     {
       viewer.spinOnce ();
     }

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}