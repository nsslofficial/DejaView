#include "my_pcl.h"
#include "my_utils.h"
#include "my_draco.h"
#include <chrono>



const std::string compression_folder    = "compressed";
const std::string input_folder          = "pcds";

int draco_quantization_parameter  = 10;
int draco_compression_level       = 10;


std::ofstream log_file_time,log_file_size;

std::chrono::steady_clock::time_point run_time_1, run_time_2, run_time_3, run_time_4, run_time_5, run_time_6, run_time_7 ,run_time_8, 
         run_time_9,run_time_10, run_time_11 , run_time_12, run_time_13, run_time_14 ,run_time_15, run_time_16;




bool compress_frames (std::string dataset_folder, std::string output_folder)
{

    std::string output_path =               dataset_folder + "/" + output_folder ;
    std::string input_path =                dataset_folder + "/" + input_folder ;
    std::string result_folder_compressed =  output_path + "/" + compression_folder;

    std::vector<std::string> direcotries_to_create; 
    direcotries_to_create.push_back(result_folder_compressed);

    std::cout << "creating output directories" << std::endl;
    if(!createDirectories(direcotries_to_create)){
        std::cout<< "fail to create directories " << std::endl; 
        return false;
    }
    std::cout << "\t"<<"output directories created successfully" << std::endl;


    for (const auto& entry : std::filesystem::directory_iterator(input_path)) 
    {
        std::cout << entry.path().filename() << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<uint8_t> compressedPCD;

        // read input cloud
        if(!readPCD(entry.path(), input_cloud))
        {
            std::cout<< "\t"<<"input pcd read fail." << std::endl;
        }

        // draco compression
        DracoEncodePointCloudToFile(input_cloud, "", draco_quantization_parameter, draco_compression_level, compressedPCD);

        // Save the encoded pcd into a file.
        std::string output_file_path = result_folder_compressed + '/' +entry.path().filename().replace_extension(".drc").string();
        writeVectorToBinaryFile(output_file_path, compressedPCD);

    }

    return true;   
}

int main(int argc, char** argv) {

    std::string dataset_folder      = std::string(argv[1]);
    std::string output_folder       = std::string(argv[2]);
    draco_quantization_parameter    = atoi(argv[3]);

    compress_frames(dataset_folder, output_folder);

    return 0;
}