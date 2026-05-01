#include "my_pcl.h"
#include "my_utils.h"
#include "my_draco.h"
#include <chrono>





const std::string decompression_folder    = "decompressed";
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


    for (const auto& entry : std::filesystem::directory_iterator(input_path)) 
    {
        std::cout << entry.path().filename() << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr decompressed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<uint8_t> compressedPCD;


        // read compressed PCD into vector
        if(!readBinaryFileToVector(entry.path(), compressedPCD))
        {
            std::cout<<"failed to read compressed file"<<std::endl;
            return false;
        }

        // Create a vector of char and copy elements from uint8Vector
        std::vector<char> charVector(compressedPCD.begin(), compressedPCD.end());

        // draco decompression
        DracoDecodePointCloudFromFile(charVector, decompressed_cloud);

        // Save the decompressed pcd 
        std::string output_file_path = result_folder_decompressed +"/"+ entry.path().filename().replace_extension(".pcd").string();
        pcl::io::savePCDFileBinary (output_file_path, *decompressed_cloud);

    }

    return true;   
}

int main(int argc, char** argv) {

    std::string dataset_folder = std::string(argv[1]);
    std::string output_folder = std::string(argv[2]);

    decompress_frames(dataset_folder, output_folder);

    return 0;
}