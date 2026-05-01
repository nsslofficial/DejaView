#ifndef MY_PCL_H
#define MY_PCL_H

#include <cmath>  
#include <vector>  
#include <string.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <omp.h>



bool initOctreeSearchObject(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
bool initKDtreeObject(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
bool initExtractObject(pcl::ExtractIndices<pcl::PointXYZ>& extract, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool setNeg );
bool extractFilter(pcl::ExtractIndices<pcl::PointXYZ>& extract, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers );
bool getKDtreeNewPoints(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& all_indices, std::vector<int>& new_indices, float squaredDistance, int K);
bool getKDtreeNearestNeighbourPoints(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& OriginalIndices, std::vector<int>& NearestNeighbourIndices, float squaredDistance, int K);
bool getOCtreeNewPoints(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& all_indices, std::vector<int>& new_indices, float squaredDistance, int K);
bool KDtree_radius_search(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& common_indices_source, std::vector<int>& common_indices_reference, float radius);
bool Octree_radius_search(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& common_indices_source, std::vector<int>& common_indices_reference, float radius);
bool getOCtreeNearestNeighbourPoints(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& OriginalIndices, std::vector<int>& NearestNeighbourIndices, float squaredDistance, int K);
bool coarseChangeOctree( pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, std::vector<int>& indices, float resolution);
bool fineChangeKdtree( pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, std::vector<int>& coarse_indices, std::vector<int>& fine_indices, float squaredDistance, int K);
bool fineChangeOctree( pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, std::vector<int>& coarse_indices, std::vector<int>& fine_indices, float squaredDistance, int K);
bool extractPoints( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, std::vector<int>& indices,  bool setNeg );
bool cropBasemap(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointIndices::Ptr inliers,  pcl::PointXYZ center_point, float radius);
bool readPCD(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud );
bool readPLY(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud );
bool readPCD_XYZI(std::string path, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud );
bool getOctreeSearchResult(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointXYZ center_point, float radius,  std::vector<int>& pointIdxRadiusSearch, std::vector<float>& pointRadiusSquaredDistance );
bool initOctreeChangeObject(pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>& octree, pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
bool getOctreeChange(pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>& octree, std::vector<int>& newPointIdxVector);
bool initInliersObject(pcl::PointIndices::Ptr inliers, std::vector<int>& indices );
bool removeZeroPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
bool runICP(pcl::PointCloud<pcl::PointXYZ>::Ptr source , pcl::PointCloud<pcl::PointXYZ>::Ptr target, int iterations, Eigen::Matrix4f& transformation );


#endif
