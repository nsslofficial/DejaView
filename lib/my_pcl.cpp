#include "my_pcl.h"



bool initOctreeSearchObject(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();
    return true;
}

bool initKDtreeObject(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    kdtree.setInputCloud (cloud);
    return true;
}

bool initExtractObject(pcl::ExtractIndices<pcl::PointXYZ>& extract, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool setNeg ){
    extract.setInputCloud (cloud);
    extract.setNegative (setNeg);
    return true;
}

bool extractFilter(pcl::ExtractIndices<pcl::PointXYZ>& extract, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers ){
    extract.setIndices(inliers);
    extract.filter(*cloud);
    return true;
}


bool getKDtreeNewPoints(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& all_indices, std::vector<int>& new_indices, float squaredDistance, int K){

    for (std::size_t point_i = 0; point_i < all_indices.size(); ++ point_i){
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if ( kdtree.nearestKSearch ((*cloud)[all_indices[point_i]], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                if ( pointNKNSquaredDistance[i] >= squaredDistance ){
                    new_indices.push_back(all_indices[point_i]);
                }
            }   
        }
    }
    return true;
}

bool KDtree_radius_search(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& common_indices_source, std::vector<int>& common_indices_reference, float radius){

    for (std::size_t point_i = 0; point_i < cloud->size(); ++ point_i){
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        if ( kdtree.radiusSearch ((*cloud)[point_i], radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            common_indices_source.push_back(point_i);
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                common_indices_reference.push_back(pointIdxNKNSearch[i]);
            }   
        }
    }
    return true;
}
bool Octree_radius_search(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& common_indices_source, std::vector<int>& common_indices_reference, float radius){

    for (std::size_t point_i = 0; point_i < cloud->size(); ++ point_i){
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        if ( octree.radiusSearch ((*cloud)[point_i], radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            common_indices_source.push_back(point_i);
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                common_indices_reference.push_back(pointIdxNKNSearch[i]);
            }   
        }
    }
    return true;
}


bool getOCtreeNewPoints(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& all_indices, std::vector<int>& new_indices, float squaredDistance, int K){

    for (std::size_t point_i = 0; point_i < all_indices.size(); ++ point_i){
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if ( octree.nearestKSearch ((*cloud)[all_indices[point_i]], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                if ( pointNKNSquaredDistance[i] >= squaredDistance ){
                    new_indices.push_back(all_indices[point_i]);
                }
            }   
        }
    }
    return true;
}

bool getOCtreeNearestNeighbourPoints(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& OriginalIndices, std::vector<int>& NearestNeighbourIndices, float squaredDistance,int K ){


    for (std::size_t point_i = 0; point_i < cloud->size(); ++ point_i)
    {
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if ( octree.nearestKSearch ((*cloud)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                    if ( pointNKNSquaredDistance[i] <= squaredDistance )
                    {
                    NearestNeighbourIndices.push_back(pointIdxNKNSearch[i]);
                    OriginalIndices.push_back(point_i);
                    }
            }   
        }
    }
    return true;
}

bool getKDtreeNearestNeighbourPoints(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& OriginalIndices, std::vector<int>& NearestNeighbourIndices, float squaredDistance,int K ){


    for (std::size_t point_i = 0; point_i < cloud->size(); ++ point_i)
    {
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if ( kdtree.nearestKSearch ((*cloud)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                    if ( pointNKNSquaredDistance[i] <= squaredDistance )
                    {
                    NearestNeighbourIndices.push_back(pointIdxNKNSearch[i]);
                    OriginalIndices.push_back(point_i);
                    }
            }   
        }
    }
    return true;
}


bool coarseChangeOctree( pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, std::vector<int>& indices, float resolution){
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud (reference_cloud);
    octree.addPointsFromInputCloud ();
    octree.switchBuffers ();
    octree.setInputCloud (target_cloud);
    octree.addPointsFromInputCloud ();
    octree.getPointIndicesFromNewVoxels(indices);
    return true;
}

bool fineChangeKdtree( pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, std::vector<int>& coarse_indices, std::vector<int>& fine_indices, float squaredDistance, int K){
    pcl::KdTreeFLANN<pcl::PointXYZ> tree_ref;
    initKDtreeObject(tree_ref, reference_cloud);
    getKDtreeNewPoints(tree_ref, target_cloud, coarse_indices, fine_indices, squaredDistance, K);
    return true;
}

bool fineChangeOctree( pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, std::vector<int>& coarse_indices, std::vector<int>& fine_indices, float squaredDistance, int K){
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree_ref (std::sqrt(squaredDistance));
    initOctreeSearchObject(tree_ref, reference_cloud);
    getOCtreeNewPoints(tree_ref, target_cloud, coarse_indices, fine_indices, squaredDistance, K);
    return true;
}



bool extractPoints( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, std::vector<int>& indices,  bool setNeg ){
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    inliers->indices = indices;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (input_cloud);
    extract.setNegative (setNeg);
    extract.setIndices (inliers);
    extract.filter(*output_cloud);
    return true;
}

bool cropBasemap(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointIndices::Ptr inliers,  pcl::PointXYZ center_point, float radius){
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    octree.radiusSearch (center_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    inliers->indices = pointIdxRadiusSearch;
    return true;
}

bool removeZeroPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z"); // Filter along the z-axis
    pass.setFilterLimits(0.0, 0.0); // Set minimum and maximum limits
    pass.setNegative (true);
    pass.filter(*cloud); // Apply the filter

    // approach one 
    // std::vector<int> indices;   
    // #pragma omp parallel 
    // {
    //     std::vector<int> privateIndices; 

    //     #pragma omp for
    //     for (std::size_t i = 0; i < cloud->size(); ++i)
    //     {
    //         pcl::PointXYZ point = (*cloud)[i];
    //         if (point.x == 0 && point.y == 0 && point.z == 0) 
    //         {
    //             privateIndices.push_back(i);
    //         }
    //     }

    //     #pragma omp critical
    //     {
    //         indices.insert(indices.end(), privateIndices.begin(), privateIndices.end());
    //     }

    // }

    // // approach two
    // std::vector<int> indices(cloud->size(),0);   
    // #pragma omp parallel 
    // {
    //     #pragma omp for
    //     for (std::size_t i = 0; i < cloud->size(); ++i)
    //     {
    //         pcl::PointXYZ point = (*cloud)[i];
    //         if (point.x != 0 && point.y != 0 && point.z != 0) 
    //         {
    //             indices[i]= i;
    //         }
    //     }
    // }
    // indices.erase(std::remove(indices.begin(), indices.end(), 0), indices.end());


    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // inliers->indices = indices;

    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud (cloud);
    // extract.setNegative (true);
    // extract.setIndices (inliers);
    // extract.filter(*cloud);


    return true;
}


    
bool readPCD(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1) {
        PCL_ERROR ("Couldn't read PCD file  \n");
        return false;
    }
    return true;
}

bool readPLY(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){
    pcl::PLYReader Reader;
    if (Reader.read(path, *cloud) == -1) {
        PCL_ERROR ("Couldn't read PLY file  \n");
        return false;
    }
    return true;
}

bool readPCD_XYZI(std::string path, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud ){
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (path, *cloud) == -1) {
        PCL_ERROR ("Couldn't read PCD file  \n");
        return false;
    }
    return true;
}


bool runICP(pcl::PointCloud<pcl::PointXYZ>::Ptr source , pcl::PointCloud<pcl::PointXYZ>::Ptr target, int iterations, Eigen::Matrix4f& transformation ){
    // pcl::PointCloud<pcl::PointXYZ> Final;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.align(*source);
    transformation = icp.getFinalTransformation ();
}

// bool getOctreeSearchResult(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, pcl::PointXYZ center_point, float radius,  std::vector<int>& pointIdxRadiusSearch, std::vector<float>& pointRadiusSquaredDistance ){
//     return true;
// }

// bool initOctreeChangeObject(pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>& octree, pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud){
//     octree.setInputCloud (reference_cloud);
//     octree.addPointsFromInputCloud ();
//     octree.switchBuffers ();
//     octree.setInputCloud (target_cloud);
//     octree.addPointsFromInputCloud ();
//     return true;
// }

// bool getOctreeChange(pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>& octree, std::vector<int>& newPointIdxVector){
//     octree.getPointIndicesFromNewVoxels(newPointIdxVector);
//     return true;
// }

// bool initInliersObject(pcl::PointIndices::Ptr inliers, std::vector<int>& indices ){
//     inliers->indices = indices;
//     return true;
// }