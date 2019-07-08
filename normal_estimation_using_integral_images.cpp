#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
//integral images is a technique of finding the sum of pixel values of any sub image of an image (sub image:image is similar to sub-matrix:matrix). It precomputes a sum matrix in which 
//each cell contains the sum of pixel values of all cell values to the left and above it in the matrix. This matrix is then used to find the sum of any sub-image.
//useful link for understanding this topic:https://computersciencesource.wordpress.com/2010/09/03/computer-vision-the-integral-image/
int
main ()
{
     // load point cloud
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

     // estimate normals
     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

     pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
     ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
     ne.setMaxDepthChangeFactor(0.02f);
     ne.setNormalSmoothingSize(10.0f);
     ne.setInputCloud(cloud);
     ne.compute(*normals);
     
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