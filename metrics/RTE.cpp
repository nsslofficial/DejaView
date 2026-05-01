#include "my_pcl.h"
#include "my_utils.h"
#include <cmath>  


int K =1;


std::ofstream log_rmse;



float findRTE (std::string GT_pose_file,  std::string ndt_pose_file)
{
  float RTE_sum = 0.0f;
  float RTE_avg = 0.0f;
  std::vector<float> GT_pose;
  std::vector<float> NDT_pose;

  // std::cout <<GT_pose_file <<std::endl;

  readTransformFromFile(GT_pose_file+".txt" , GT_pose);
  readTransformFromFile(ndt_pose_file+".txt" , NDT_pose);

  // std::cout <<GT_pose.size() <<std::endl;
  // std::cout <<"here" <<std::endl;


  for (int i =0; i< GT_pose.size() ; i+=7){
  // std::cout <<"here" <<std::endl;
    RTE_sum += sqrt(std::pow(GT_pose[i]- NDT_pose[i],2) + 
                    std::pow(GT_pose[i+1] - NDT_pose[i+1],2)+
                     +  std::pow(GT_pose[i+2] - NDT_pose[i+2],2));
    // std::cout << RTE_sum <<std::endl;
  }

  RTE_avg = RTE_sum / (GT_pose.size() /3) ;

  std::cout << RTE_avg << std::endl;

  return RTE_avg;
}



int main(int argc, char** argv)
{ 
  std::string data_folder = argv[1];
  std::string originalPoseFile = data_folder + '/' + argv[2];
  std::string NdtPoseFile = data_folder + '/' + argv[3];


  findRTE(originalPoseFile, NdtPoseFile);


}