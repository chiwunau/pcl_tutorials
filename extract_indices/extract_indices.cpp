#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::visualization::CloudViewer viewer1 ("Simple Cloud Viewer1");
  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_print_a;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_print_b;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_new (cloud_filtered->makeShared());
  for (size_t k = 0; k < (*cloud_filtered_new).points.size (); ++k) {  
      	cloud_filtered_new->points[k].r = 255;  
      	cloud_filtered_new->points[k].g = 255;  
      	cloud_filtered_new->points[k].b = 255;  
      }  

      int j = 0;
  while (cloud_filtered->points.size () > 0.3 * nr_points)
    {

      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
       if (inliers->indices.size () == 0)
       	{
       	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
       	  break;
       	}

      // Extract the inliers
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_p);
      
      
      if(j==0){      
	cloud_print_a = (*cloud_p).makeShared();
	for (size_t k = 0; k < cloud_print_a->points.size (); ++k) {
	  cloud_print_a->points[k].r = 255;  
	  cloud_print_a->points[k].g = 0;  
	  cloud_print_a->points[k].b = 0;  
	}
      }
	else{
	  cloud_print_b = (*cloud_p).makeShared();
	  for (size_t k = 0; k < cloud_print_a->points.size (); ++k) {
	  cloud_print_b->points[k].r = 0;  
	  cloud_print_b->points[k].g = 255;  
	  cloud_print_b->points[k].b = 0;  
	  }
	}

       std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

       std::stringstream ss;
       ss << "table_scene_lms400_plane_" << i << ".pcd";
       writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);


       // Create the filtering object
       extract.setNegative (true);
       extract.filter (*cloud_f);
       cloud_filtered.swap (cloud_f);
       i++;
       j++;
    }
    viewer1.showCloud(cloud_filtered_new,"c");
  viewer1.showCloud(cloud_print_a,"a");
  viewer1.showCloud(cloud_print_b,"b");

  
  
  // pcl::visualization::CloudViewer viewer2 ("Simple Cloud Viewer2");
  //pcl::visualization::CloudViewer viewer3 ("Simple Cloud Viewer3");



   //viewer3.showCloud(cloud_f);

   while(!viewer1.wasStopped())
     {};
  

  return (0);
}
