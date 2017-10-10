#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//for projection
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  //TODO: set input path
 //INPUT_PATH_TO_COMPLETE should en with .ply
  if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal> (INPUT_PATH_TO_COMPLETE, *inputcloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  
  int PLANE_THRESHOLD = 1000; //if a plane is detected but has less than PLANE_THRESHOLD points, it will be filtered out and the algorithm will stop 
  int MAX_NUM_PLANE =  5;
  
  int num_plane = 0; // keep track of nb of plane for MAX_NUM_PLANE
  int plane_inliers = PLANE_THRESHOLD+1;//initialize so that we at least enter once in the loop, keep track of nb of points in the plane for PLANE_THRESHOLD
  bool PROJECT_TO_PLANE = false;
  bool COLOR_INLIER = true;
  //color to use for the planes : red,orange,yellow,green,blueish green, lightblue, blue, purple, pink    (http://www.rapidtables.com/web/color/RGB_Color.htm)
  int[] rcolor={255,255,255,0  ,0  ,0  ,0  ,127,255}//9 colors 
  int[] gcolor={0  ,128,255,255,255,255,0  ,0  ,0  }
  int[] bcolor={0  ,0  ,0  ,0  ,128,255,255,255,255}
  int color_index = 0;

  //the SAC segmentation runs on RANSAC and find the best plane for the given points 
  //At each loop , the following find the best plane for the points, store it and then remove the inliers from the point cloud. This is to make sure that for the next iteration,
//  we find a new plane. This new plane will have fewer inliers. At some point, the stop condition will be reached. 
  while(plane_inliers < PLANE_THRESHOLD || (num_plane> MAX_NUM_PLANE))
  {
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  // Create the segmentation object
	  pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
	  // Optional
	  seg.setOptimizeCoefficients (true);
	  // Mandatory
	  seg.setModelType (pcl::SACMODEL_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setDistanceThreshold (0.01);

	  seg.setInputCloud (inputcloud);
	  seg.segment (*inliers, *coefficients);
	  
	  if (inliers->indices.size () == 0)
	  {
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return (-1);
	  }
	  //create inliers cloud 
	    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inlierscloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        for (size_t i = 0; i < inliers->indices.size (); ++i)
		 {
			inlierscloud->insert(inputcloud->points[inliers->indices[i]]);
		 }
		//At this point we have information about the plane 
		//set this boolean to true if you want to project inliers to the plane 
		if(PROJECT_TO_PLANE)
		{
			pcl::ProjectInliers<pcl::PointXYZ> proj;
			proj.setModelType (pcl::SACMODEL_PLANE);
			proj.setInputCloud (inlierscloud);
			proj.setModelCoefficients (coefficients);
			proj.filter (*inlierscloud);
		}
		//set this boolean to true if you want to color the points from the plane 
		if(COLOR_INLIER)
		{
			 for (size_t i = 0; i < inliers->indices.size (); ++i)
			 {
				 inputcloud->points[inliers->indices[i]].r=rcolor[color_index];
				 inputcloud->points[inliers->indices[i]].g=gcolor[color_index];
				 inputcloud->points[inliers->indices[i]].b=bcolor[color_index];
			 }
		}
		//save all the transformed points to the outputcloud and remove them from inputcloud
		//TODO: check that the indexes are in increasing order, otherwise try 
		//std::sort(inliers->indices.begin(), inliers->indices.end())
		for (size_t i = 0; i < inliers->indices.size (); ++i)
		{
			int point_index= inliers->indices[i]-i;
			//add to output
			outputcloud->insert(inputcloud->points[point_index]);
			//remove from inputcloud
			inputcloud->points.erase(point_index);
		}
	//update indexes 
	plane_inliers= indices.size ();
	color_index=(color_index+1)%10;
	num_plane=num_plane+1;
  }
  	//save all the remaining input points to the outputcloud
	for (size_t i = 0; i < inputcloud->width; ++i)
	{
		//color points that are not in a plane in white 
		if(COLOR_INLIER)
		{
			inputcloud->points[i].r=255;
			inputcloud->points[i].g=255;
			inputcloud->points[i].b=255;
		}
		outputcloud->insert(inputcloud->points[i]);
	}
	//TODO: write where you want 
	//OUTPUT_PATH_TO_COMPLETE should en with .ply
	pcl::io::savePLYFileASCII (OUTPUT_PATH_TO_COMPLETE, *outputcloud); 
  return (0);
}