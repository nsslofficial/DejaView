#include "my_pcl.h"
#include "my_utils.h"
#include "my_draco.h"
#include <chrono>
#include <omp.h>
#include <filesystem>

const std::string decompression_folder  = "decompressed";
const std::string input_folder          = "compressed";

bool decompress_frames (std::string dataset_folder, std::string output_folder)
{
    std::string output_path =               dataset_folder + "/" + output_folder ;
    std::string input_path =                output_path + "/" + input_folder ;
    std::string result_folder_decompressed =  output_path + "/" + decompression_folder;

    std::vector<std::string> direcotries_to_create; 
    direcotries_to_create.push_back(result_folder_decompressed);

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

        pcl::PointCloud<pcl::PointXYZ>::Ptr decompressed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<uint8_t> compressedPCD;

        if(!readBinaryFileToVector(current_path.string(), compressedPCD))
        {
            #pragma omp critical
            std::cout<<"failed to read compressed file"<<std::endl;
            continue;
        }

        std::vector<char> charVector(compressedPCD.begin(), compressedPCD.end());

        DracoDecodePointCloudFromFile(charVector, decompressed_cloud);

        std::string output_file_path = result_folder_decompressed +"/"+ current_path.filename().replace_extension(".pcd").string();
        pcl::io::savePCDFileBinary (output_file_path, *decompressed_cloud);
    }

    auto stop_overall = std::chrono::high_resolution_clock::now();
    auto duration_overall = std::chrono::duration_cast<std::chrono::microseconds>(stop_overall - start_overall);
    std::cout << "<<<TOTAL_TIME:" << duration_overall.count() << " microseconds>>>" << std::endl;

    return true;   
}

int main(int argc, char** argv) {
    if(argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <dataset_folder> <output_folder>\n";
        return 1;
    }
    std::string dataset_folder = std::string(argv[1]);
    std::string output_folder = std::string(argv[2]);

    decompress_frames(dataset_folder, output_folder);

    return 0;
}
