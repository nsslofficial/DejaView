#include "my_pcl.h"
#include "my_utils.h"
#include <chrono>
#include <omp.h>
#include <filesystem>

int K =1;

std::ofstream log_rmse;

float computeSingleFrameRMSE (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, int K)
{
  float rmse = 0.0f;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud (cloud_target);
  
  #pragma omp parallel for reduction(+:rmse)
  for (int point_i = 0; point_i < (int)cloud_source->size(); ++ point_i)
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

  #pragma omp parallel for reduction(+:avg_rmse) schedule(dynamic)
  for (int i=0; i< (int)filenames.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);

    // read input cloud
    if(!readPCD(source_pcd_folder + '/' + filenames[i], cloud_source))
    {
      if (!readPLY(source_pcd_folder  + '/' + filenames[i], cloud_source))
      {
        #pragma omp critical
        std::cout<< "\t"<<"input read fail." << std::endl;
      }
    }

    // read input cloud
    if(!readPCD(target_pcd_folder  + '/' + filenames[i], cloud_target))
    {
      if (!readPLY(target_pcd_folder  + '/' + filenames[i], cloud_target)) 
      {
        #pragma omp critical
        std::cout<< "\t"<<"input read fail." << std::endl;
      }
    }
  
    removeZeroPoints(cloud_source);
    removeZeroPoints(cloud_target);
    avg_rmse += computeSingleFrameRMSE (cloud_source, cloud_target, K);
  }
  avg_rmse = avg_rmse/ (float) filenames.size();

  return avg_rmse;
}

bool computeChamferDistnce(std::string source_pcd_folder,  std::string target_pcd_folder){

  std::vector<std::string> filenames;
  for (const auto & entry : std::filesystem::directory_iterator(source_pcd_folder)){
    filenames.push_back(entry.path().filename().string());
  }

  float rmseOriginalToReconstructed = computeRMSE( source_pcd_folder, target_pcd_folder, filenames);
  float rmseReconstructedToOriginal = computeRMSE( target_pcd_folder, source_pcd_folder, filenames);
  float chamferDistance = (rmseOriginalToReconstructed + rmseReconstructedToOriginal) /2;

  std::cout << rmseOriginalToReconstructed << "\t" << rmseReconstructedToOriginal << "\t"<< chamferDistance << std::endl;

  return true;
}

int main(int argc, char** argv)
{
  if(argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <data_path> <originalPCDFolder> <ReconstructedPCDFolder>\n";
    return 1;
  }
  std::string data_path = argv[1];
  std::string originalPCDFolder = argv[2];
  std::string ReconstructedPCDFolder = argv[3];

  std::string source_pcd_folder = data_path + '/' + originalPCDFolder;
  std::string target_pcd_folder = data_path + '/'+ ReconstructedPCDFolder;

  computeChamferDistnce(source_pcd_folder, target_pcd_folder );
}
