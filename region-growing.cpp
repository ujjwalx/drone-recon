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

//for curvature computation
#include <vector>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

//for region growing segmentation
//#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
//Create point cloud from indices
#include <pcl/filters/extract_indices.h>
int
 main (int argc, char** argv)
{
  std::string dataDir = "/home/finde/Desktop/plane_constraint/data/";

  bool FILTER_DISTANCE = false;
  bool PROJECT_TO_PLANE = false;
  bool COLOR_INLIER= true;
   //color to use for the planes : red,orange,yellow,green,blueish green, lightblue, blue, purple, pink    (http://www.rapidtables.com/web/color/RGB_Color.htm)
  int[] rcolor={255,255,255,0  ,0  ,0  ,0  ,127,255}//9 colors 
  int[] gcolor={0  ,128,255,255,255,255,0  ,0  ,0  }
  int[] bcolor={0  ,0  ,0  ,0  ,128,255,255,255,255}
  int color_index = 0;
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputcloud_noNormals(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputleftbehindcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  pcl::copyPointCloud(*inputcloud,*inputcloud_noNormals);
  //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(dataDir + "fused.ply", *inputcloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  //COMPUTE THE CURVATURE 
  /*
// Setup the principal curvatures computation
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

  // Provide the original point cloud (without normals)
  principal_curvatures_estimation.setInputCloud (inputcloud_noNormals);

  // Provide the point cloud with normals
  principal_curvatures_estimation.setInputNormals (inputcloud);

  // Use the same KdTree from the normal estimation
  principal_curvatures_estimation.setSearchMethod (tree);
  principal_curvatures_estimation.setRadiusSearch (1.0);

  // Actually compute the principal curvatures
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
  principal_curvatures_estimation.compute (*principal_curvatures); */

  //Run Region Growing Segmentation Algorithm
  if(FILTER_DISTANCE)
  {
	  //filter the z distance 
	  pcl::IndicesPtr indices (new std::vector <int>);
      pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
      pass.setInputCloud (inputcloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 1.0);
      pass.filter (*indices);
  }
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> reg;
  reg.setMinClusterSize (1000);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (inputcloud_noNormals);
  if(FILTER_DISTANCE)
  {
     reg.setIndices (indices);
  }
  reg.setInputNormals (inputcloud);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  
  //Cluster size : clusters.size ()
  //Nb points 1st cluster : clusters[0].indices.size ()
  //Get colored Regions: pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr output_cloud = reg.getColoredCloud ();
  
  cout << "Cluster number: " << clusters.size() <<endl;
  //Now that we have the clusters, we can fit a plane for each of them 
   pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
   //Initialize the outputleftbehindcloud (point cloud that keeps track of all the points that are in no clusters) with the full point cloud
   pcl::copyPointCloud(inputcloud,outputleftbehindcloud)
  for(int i=0; i< clusters.size(); i++)
  {
	  //create the cluster_input_cloud, color it and project it, finally, add it to the output_cloud
	  //also remove those points from the original input cloud and store the result in outputleftbehindcloud
	  //so that we keep track of the points that doesn't belongs to any cluster
	  
	  //create cluster_input_cloud which contains the inliers of the cluster 
	  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster_input_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	  extract.setInputCloud (inputcloud);
      extract.setIndices (clusters[i]);
      extract.setNegative (false);
      extract.filter (*cluster_input_cloud);
		
	  //keep track of the points that doesn't belongs to any cluster
	  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputleftbehindcloudtemp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	  extract.setInputCloud(outputleftbehindcloud);
	  extract.setIndices(clusters[i]);
	  extract.setNegative(true);
	  extract.filter(*outputleftbehindcloudtemp)
	  outputleftbehindcloud=outputleftbehindcloudtemp
	  
	  //compute the plane of the cluster 
	  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
      pcl::SACSegmentation <pcl::PointXYZRGBNormal> seg;
      // Optional
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.1);
      seg.setInputCloud(cluster_input_cloud);
      seg.segment(*inliers_plane, *coefficients_plane);
	  
	  //Compute the points that belongs to the cluster AND the plane, disregard the ones that do not belong to the plane
	  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_plane_inliers(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	  extract.setInputCloud (cluster_input_cloud);
      extract.setIndices (inliers_plane);
      extract.setNegative (false);
      extract.filter (*output_plane_inliers);
		
	    if (PROJECT_TO_PLANE) {
            pcl::ProjectInliers <pcl::PointXYZRGBNormal> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(output_plane_inliers);
            proj.setModelCoefficients(coefficients);
            proj.filter(*output_plane_inliers);
		}
	
	    if(COLOR_INLIER)
		{
			for(int j=0; j< output_plane_inliers.width; j++)
			{
				output_plane_inliers->points[j].r = rcolor[color_index];
				output_plane_inliers->points[j].g = gcolor[color_index];
				output_plane_inliers->points[j].b = bcolor[color_index];
			}
		}
		//update output cloud with only inliers from plane (disregard the others)
		
		*outputcloud+=*output_plane_inliers
		
		//update loop indexes 
		color_index=(color_index+1)%10;
  }
//Add the points that do not belong to any clusters 
	if (COLOR_INLIER) {
		for (size_t i = 0; i < outputleftbehindcloud->width; ++i) {
			//color points that are not in a plane in white
			outputleftbehindcloud->points[i].r = 255;
			outputleftbehindcloud->points[i].g = 255;
			outputleftbehindcloud->points[i].b = 255;
		}
	}
	*outputcloud+=*outputleftbehindcloud
    pcl::io::savePLYFile(dataDir + "NEWfused_region.ply", *outputcloud);
 
  return (0);
}