#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
//integral images is a technique of finding the sum of pixel values of any sub image of an image (sub image:image is similar to sub-matrix:matrix). It precomputes a sum matrix in which 
//each cell contains the sum of pixel values of all cell values to the left and above it in the matrix. This matrix is then used to find the sum of any sub-image.
//useful link for understanding this topic:https://computersciencesource.wordpress.com/2010/09/03/computer-vision-the-integral-image/
int
main (int argc, char** argv)
{
     // load point cloud
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
   cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
  
  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

     // estimate normals
     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
     ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
     ne.setMaxDepthChangeFactor(0.02f);
     ne.setNormalSmoothingSize(10.0f);
     ne.setInputCloud(cloud);
     ne.compute(*normals);
    //   cloud.resize (cloud.width * cloud.height);
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
     
     viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);

     while (!viewer.wasStopped ())
     {
       viewer.spinOnce ();
     }

     return 0;
}