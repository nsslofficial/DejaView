#ifndef MY_VARIABLES_H
#define MY_VARIABLES_H

// comman variables 
const float distance_threshold =4;
const float lidar_range =200.0;
const int K=1; //number of nn in KDtree result
const int buffer_size = 1024*1024;
const int preset = 9;
const int draco_quantization_parameter = 10;
const int draco_compression_level = 10;
bool filter = true;
float resolution =0.1;
float NN_distance =0.1;
int icp_iterations = 100;
const std::string output_extension = ".lc";
const char seperator = ','; // for read world ',' ,for carla ','
int multiple_day_flag = 0;

// input folder/file names 
const std::string basemap_pcd_name = "basemap/basemap.pcd";
const std::string pose_file_name ="pose.txt";
const std::string pcds_folder_name ="pcds";
const std::string icp_transforms_file ="icp_transformations.txt";


// output folder names (compression)
const std::string remove_indices_folder = "remove_indices";
const std::string remove_indices_folder_sorted = "remove_indices_sorted";
const std::string remove_indices_folder_compressed = "remove_indices_compressed";

const std::string add_indices_folder = "add_indices";
const std::string add_indices_folder_sorted = "add_indices_sorted";
const std::string add_indices_folder_compressed= "add_indices_compressed";
const std::string indices_folder_compressed= "indices_compressed";
const std::string result_folder= "compressed";

const std::string add_pcd_folder = "add_pcds";
const std::string add_ply_folder = "add_plys";

const std::string subset_pcd_folder = "subset_pcds_0.1";
const std::string add_pcd_folder_compressed = "add_pcds_compressed_draco_"+std::to_string(draco_quantization_parameter) ;

// output folder names (decompression)
std::string remove_indices_sorted_decompressed_folder ="remove_indices_sorted_decompressed";
std::string add_indices_sorted_decompressed_folder ="add_indices_sorted_decompressed";

#endif



