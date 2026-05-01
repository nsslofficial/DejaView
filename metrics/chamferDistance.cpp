#include "my_pcl.h"
#include "my_utils.h"
#include <chrono>

int K =1;


std::ofstream log_rmse;


float computeSingleFrameRMSE (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, int K)
{

  float rmse = 0.0f;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud (cloud_target);
  for (std::size_t point_i = 0; point_i < cloud_source->size(); ++ point_i)
  {
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if ( tree.nearestKSearch ((*cloud_source)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      {
        rmse += pointNKNSquaredDistance[i];
      }
      
    }
  }

  rmse = std::sqrt (rmse / (float)cloud_source->size());

  return rmse;

}


float computeRMSE (std::string source_pcd_folder,  std::string target_pcd_folder, std::vector<std::string> filenames)
{
  float avg_rmse = 0.0f;
  float rmse = 0.0f;

  #pragma omp parallel for reduction(+:avg_rmse)
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
            std::cout<<cloud_target->size()<<std::endl;

    }
  
    removeZeroPoints(cloud_source);
    removeZeroPoints(cloud_target);
    avg_rmse+= computeSingleFrameRMSE (cloud_source, cloud_target, K);
    // rmse = computeSingleFrameRMSE (cloud_source, cloud_target, K);
    // log_rmse << filenames[i] << "," << rmse << std::endl;
    // avg_rmse += rmse;


  }
  avg_rmse = avg_rmse/ (float) filenames.size();

  return avg_rmse;
}

bool computeChamferDistnce(std::string source_pcd_folder,  std::string target_pcd_folder){

  std::vector<std::string> filenames;
  for (const auto & entry : std::filesystem::directory_iterator(source_pcd_folder)){
    filenames.push_back(entry.path().filename());
  }

  // std::cout<< "total pcds: " << filenames.size() <<std::endl;

  float rmseOriginalToReconstructed = computeRMSE( source_pcd_folder, target_pcd_folder, filenames);
  float rmseReconstructedToOriginal = computeRMSE( target_pcd_folder, source_pcd_folder, filenames);
  float chamferDistance = (rmseOriginalToReconstructed + rmseReconstructedToOriginal) /2;

  // std::cout << "rmseOriginalToReconstructed: " << rmseOriginalToReconstructed << std::endl;
  // std::cout << "rmseReconstructedToOriginal: " << rmseReconstructedToOriginal << std::endl;
  // std::cout << "chamferDistance: " << chamferDistance << std::endl;
  std::cout << rmseOriginalToReconstructed << "\t" << rmseReconstructedToOriginal << "\t"<< chamferDistance << std::endl;


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


  computeChamferDistnce(source_pcd_folder, target_pcd_folder );


}