#include "my_pcl.h"
#include "my_utils.h"
#include <cmath>  
#include <omp.h>

int K =1;

std::ofstream log_rmse;

float findRTE (std::string GT_pose_file,  std::string ndt_pose_file)
{
  float RTE_sum = 0.0f;
  float RTE_avg = 0.0f;
  std::vector<float> GT_pose;
  std::vector<float> NDT_pose;

  readTransformFromFile(GT_pose_file+".txt" , GT_pose);
  readTransformFromFile(ndt_pose_file+".txt" , NDT_pose);

  int num_poses = GT_pose.size() / 7;
  
  #pragma omp parallel for reduction(+:RTE_sum)
  for (int j = 0; j < num_poses; j++){
    int i = j * 7;
    RTE_sum += sqrt(std::pow(GT_pose[i]- NDT_pose[i],2) + 
                    std::pow(GT_pose[i+1] - NDT_pose[i+1],2)+
                     +  std::pow(GT_pose[i+2] - NDT_pose[i+2],2));
  }

  // Maintaining the exact same logic as original RTE.cpp
  RTE_avg = RTE_sum / (GT_pose.size() / 3) ;

  std::cout << RTE_avg << std::endl;

  return RTE_avg;
}

int main(int argc, char** argv)
{ 
  if(argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <data_folder> <originalPoseFile> <NdtPoseFile>\n";
    return 1;
  }
  std::string data_folder = argv[1];
  std::string originalPoseFile = data_folder + '/' + argv[2];
  std::string NdtPoseFile = data_folder + '/' + argv[3];

  findRTE(originalPoseFile, NdtPoseFile);
}
