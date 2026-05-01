#include "my_pcl.h"
#include "my_utils.h"
#include <pcl/compression/octree_pointcloud_compression.h>
#include <chrono>
using namespace std::chrono;

const std::string compression_folder    = "compressed";
const std::string input_folder          = "pcds";

bool VoxelGridDownDownSampling  = false;
int i_frame                     = 1;
float octree_resolution                = 0.1;
float point_resolution                = 0.001;



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

    std::vector<std::filesystem::directory_entry> entries;
    for (const auto& entry : std::filesystem::directory_iterator(input_path)) {
        entries.push_back(entry);
    }

    // for (const auto& entry : std::filesystem::directory_iterator(input_path)) {

    auto start_overall = high_resolution_clock::now();
    #pragma omp parallel for
    for (size_t i = 0; i < entries.size(); ++i) {
        const auto& entry = entries[i];

        std::cout << entry.path().filename() << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        std::stringstream compressedData;


        // read input cloud
        if(!readPCD(entry.path(), input_cloud))
        {
            std::cout<< "\t"<<"input pcd read fail." << std::endl;
        }

        removeZeroPoints(input_cloud);

        // octree compression
        pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression (pcl::io::MANUAL_CONFIGURATION,false, point_resolution, octree_resolution, VoxelGridDownDownSampling, i_frame, false, 8);
        octree_compression.encodePointCloud(input_cloud, compressedData);

        // Save the encoded pcd into a file.
        std::string output_file_path = result_folder_compressed + '/' +entry.path().filename().replace_extension(".oct").string();
        writeStringStreamToBinaryFile( output_file_path, compressedData);

    }

    auto stop_overall = high_resolution_clock::now();
    auto duration_overall = duration_cast<microseconds>(stop_overall - start_overall);
    std::cout << "<<<TOTAL_TIME:" << duration_overall.count() << ">>>" << std::endl;

    return true;   
}

int main(int argc, char** argv) {

    std::string dataset_folder      = std::string(argv[1]);
    std::string output_folder       = std::string(argv[2]);
    VoxelGridDownDownSampling       = stringToBool(std::string(argv[3]));
    i_frame                         = atoi(argv[4]);
    octree_resolution               = atof(argv[5]);
    point_resolution                = atof(argv[6]);

    compress_frames (dataset_folder,  output_folder);

    return 0;
}