#include "my_pcl.h"
#include "my_utils.h"
#include <pcl/features/normal_3d.h>
#include <cmath>
#include <algorithm>
#include <chrono>

std::ofstream log_rmse;


bool computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int neighbours)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());


  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(neighbours);

  // ne.setRadiusSearch(0.1);// Can also use radius to find nearest neighbours instead of specifying number

  ne.compute(*normals);

  return true;
          
}

float computeMaxNNDisTwoPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
  float dist = 0;
  int K = 1;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud2);
  for (std::size_t point_i = 0; point_i < cloud1->size(); ++ point_i)
  {
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if ( tree.nearestKSearch ((*cloud1)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      {
        if (dist < pointNKNSquaredDistance[i]){
          dist = pointNKNSquaredDistance[i];
        }
      }
    }
  }
  return dist;
}

float computeMaxNNDisOnePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1)
{
  float dist = 0;
  int K = 2;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud1);
  for (std::size_t point_i = 0; point_i < cloud1->size(); ++ point_i)
  {
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if ( tree.nearestKSearch ((*cloud1)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      {
        if (dist < pointNKNSquaredDistance[i]){
          dist = pointNKNSquaredDistance[i];
        }
      }
      // std::cout << dist << std::endl;
    }
  }
  return dist;
}


bool printPoint (pcl::PointXYZ& point ){
  std::cout << point.x<< ",   " <<point.y << ",   " << point.z  <<std::endl;
  return true;
}

bool printNormal (pcl::Normal& normal){
  std::cout << normal.normal_x<< ",   " <<normal.normal_y << ",   " << normal.normal_z  <<std::endl;
  return true;
}


pcl::PointXYZ computeDifference (pcl::PointXYZ& point1, pcl::PointXYZ& point2 ){
  pcl::PointXYZ difference;
  difference.x = point1.x - point2.x;
  difference.y = point1.y - point2.y;
  difference.z = point1.z - point2.z;
  return difference;

}

float dot_product (pcl::PointXYZ& point_difference ,pcl::Normal& normal){
  float dot_p = point_difference.x * normal.normal_x + point_difference.y * normal.normal_y + point_difference.z * normal.normal_z;
  return dot_p*dot_p;
}

bool computeMeanSquaredErrorAndMaxNNDist(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, float& mse, float& max_dist)
{

  pcl::PointCloud<pcl::Normal>::Ptr cloud1_normals (new pcl::PointCloud<pcl::Normal>);
  computeNormals(cloud1, cloud1_normals, 24); // setting the numbers of NN to 12 because OctSqueeze use the same number

  float dist = 0.0;
  float squared_error = 0.0f;
  int K = 1;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud (cloud2);
  for (std::size_t point_i = 0; point_i < cloud1->size(); ++ point_i)
  {

    if (std::isnan((*cloud1_normals)[point_i].normal_x)) {
      continue; // Skip this point because a reliable normal couldn't be formed
    }  


    // std::cout << point_i << std::endl;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K); 

    if ( tree.nearestKSearch ((*cloud1)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {


      for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      {
        
        if (dist < pointNKNSquaredDistance[i]){
          dist = pointNKNSquaredDistance[i];
        }

        pcl::PointXYZ difference = computeDifference((*cloud2)[pointIdxNKNSearch[i]],(*cloud1)[point_i]);
        // pcl::PointXYZ difference = computeDifference((*cloud1)[point_i], (*cloud2)[pointIdxNKNSearch[i]]);

        // printPoint((*cloud2)[pointIdxNKNSearch[i]]);
        // printPoint((*cloud1)[point_i]);
        // printPoint(difference);
        // printNormal((*cloud1_normals)[point_i]);

        float squared_dot_product = dot_product(difference, (*cloud1_normals)[point_i]);
        // std::cout << squared_dot_product <<std::endl;

        squared_error+=squared_dot_product;
        // std::cout << squared_error <<std::endl;

      }
    }
  }
  // std::cout << squared_error <<std::endl;
  mse = squared_error/ (float) cloud1->size();
  max_dist = dist;
  return true;
}


float computePSNR (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{

  float mse, max_dist;
  computeMeanSquaredErrorAndMaxNNDist(cloud_source, cloud_target, mse, max_dist);
  // std::cout << max_dist <<std::endl;
  max_dist = computeMaxNNDisOnePCD(cloud_source);
  // std::cout << max_dist <<std::endl;
  // std::cout << mse <<std::endl;

  float psnr = 10 * std::log10(max_dist/mse);
  // std::cout << psnr <<std::endl;

  return psnr;

}


float computePSNRSymetricSingleFrame(std::string source_pcd_folder,  std::string target_pcd_folder, std::vector<std::string> filenames)
{
  float avg_psnr1 = 0.0f;
  float avg_psnr2 = 0.0f;
  float avg_psnr = 0.0f;
  float psnr = 0.0f;

  // #pragma omp parallel for reduction(+:avg_psnr1, avg_psnr2,avg_psnr)
  for (int i=0; i< filenames.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);


    // read input cloud
    if(!readPCD(source_pcd_folder + '/' + filenames[i], cloud_source))
    {
      if (!readPLY(source_pcd_folder  + '/' + filenames[i], cloud_source))
      {
        std::cout<< "\t"<<"input read fail." << std::endl;
      }
    }

    // read input cloud
    if(!readPCD(target_pcd_folder  + '/' + filenames[i], cloud_target))
    {
      if (!readPLY(target_pcd_folder  + '/' + filenames[i], cloud_target))
      {
        std::cout<< "\t"<<"input read fail." << std::endl;
      }
    }
  
    // std::cout << cloud_source->size() <<std::endl;
    // std::cout << cloud_target->size() <<std::endl;
    removeZeroPoints(cloud_source);
    removeZeroPoints(cloud_target);
    // std::cout << cloud_source->size() <<std::endl;
    // std::cout << cloud_target->size() <<std::endl;
    // exit(0);

    float psnr1 = computePSNR (cloud_source, cloud_target);
    float psnr2 = computePSNR (cloud_target, cloud_source);

    // std::cout <<  psnr1 <<std::endl;
    avg_psnr1 +=  psnr1;
    avg_psnr2 +=  psnr2;

    psnr =  std::min(psnr1, psnr2);
    avg_psnr +=  psnr;



    // std::cout << psnr <<std::endl;
    // rmse = computeSingleFrameRMSE (cloud_source, cloud_target, K);
    // log_rmse << filenames[i] << "," << rmse << std::endl;
    // avg_rmse += rmse;


  }
  avg_psnr1 = avg_psnr1/ (float) filenames.size();
  avg_psnr2 = avg_psnr2/ (float) filenames.size();
  avg_psnr = avg_psnr/ (float) filenames.size();


  std::cout << avg_psnr1 << "\t" << avg_psnr2 << "\t"<< avg_psnr << std::endl;


  return avg_psnr;
}

bool computePSNRSymetric(std::string source_pcd_folder,  std::string target_pcd_folder){

  std::vector<std::string> filenames;
  for (const auto & entry : std::filesystem::directory_iterator(source_pcd_folder)){
    filenames.push_back(entry.path().filename());
  }

  // std::cout<< "total pcds: " << filenames.size() <<std::endl;

  float psnr = computePSNRSymetricSingleFrame(source_pcd_folder, target_pcd_folder, filenames);
  // std::cout << "PSNR: " << psnr << std::endl;
  // std::cout << psnr << std::endl;


  return true;
}

int main(int argc, char** argv)
{
  std::string data_path = argv[1];
  std::string originalPCDFolder = argv[2];
  std::string ReconstructedPCDFolder = argv[3];

  std::string source_pcd_folder = data_path + '/' + originalPCDFolder;
  std::string target_pcd_folder = data_path + '/'+ ReconstructedPCDFolder;

  // log_rmse.open(data_path+"/log_rmse.csv");
  // log_rmse << "frame" << "," << "rmse" << std::endl;


  computePSNRSymetric(source_pcd_folder, target_pcd_folder );


}