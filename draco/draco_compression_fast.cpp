#include "my_pcl.h"
#include "my_utils.h"
#include "my_draco.h"
#include <chrono>
#include <omp.h>
#include <filesystem>

const std::string compression_folder    = "compressed";
const std::string input_folder          = "pcds";

int draco_quantization_parameter  = 10;
int draco_compression_level       = 10;

std::ofstream log_file_time,log_file_size;

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

    std::vector<std::string> files;
    for (const auto& entry : std::filesystem::directory_iterator(input_path)) 
    {
        files.push_back(entry.path().string());
    }

    auto start_overall = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)files.size(); i++) 
    {
        std::filesystem::path current_path(files[i]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<uint8_t> compressedPCD;

        if(!readPCD(current_path.string(), input_cloud))
        {
            #pragma omp critical
            std::cout<< "\t"<<"input pcd read fail." << std::endl;
            continue;
        }

        DracoEncodePointCloudToFile(input_cloud, "", draco_quantization_parameter, draco_compression_level, compressedPCD);

        std::string output_file_path = result_folder_compressed + '/' + current_path.filename().replace_extension(".drc").string();
        writeVectorToBinaryFile(output_file_path, compressedPCD);
    }

    auto stop_overall = std::chrono::high_resolution_clock::now();
    auto duration_overall = std::chrono::duration_cast<std::chrono::microseconds>(stop_overall - start_overall);
    std::cout << "<<<TOTAL_TIME:" << duration_overall.count() << " microseconds>>>" << std::endl;

    return true;   
}

int main(int argc, char** argv) {
    if(argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <dataset_folder> <output_folder> <draco_quantization_parameter>\n";
        return 1;
    }
    std::string dataset_folder      = std::string(argv[1]);
    std::string output_folder       = std::string(argv[2]);
    draco_quantization_parameter    = atoi(argv[3]);

    compress_frames(dataset_folder, output_folder);

    return 0;
}
