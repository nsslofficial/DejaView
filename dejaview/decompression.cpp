
#include "my_lzma.h"
#include "my_pcl.h"
#include "my_utils.h"
#include "my_draco.h"
#include "my_variables.h"
#include <omp.h>


std::ofstream log_file_time;

float run_time_1, run_time_2, run_time_3, run_time_4, run_time_5, run_time_6, run_time_7, run_time_8, run_time_9, run_time_10, run_time_11 ;


bool DecompressVectorLzma(std::vector<uint8_t>& compressedData, std::vector<int>& outputVector){

    // Deompression
    std::vector<uint8_t> decompressedData;
    if(!decompressVector(compressedData,decompressedData, buffer_size, preset )){
        std::cout << "Decompression failed" <<std::endl;
        return false;
    }

    outputVector.resize(decompressedData.size() / sizeof(int));
    memcpy(outputVector.data(), decompressedData.data(), decompressedData.size());

    return true;

}

bool recoverFromDiffVector(std::vector<int>& outputVector){

    // recover vector from diff vector 
    for (int i = 1; i < outputVector.size(); ++i) {
        outputVector[i] = outputVector[i - 1] - outputVector[i];
    }

    return true;
}


bool recover(std::string reference_day_path,std::string reference_frame,std::string recover_day_path, 
            std::string recover_frame, std::string input_folder, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr basemap, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_recovered, 
            std::vector<int>& reference_cloud_indices, std::vector<int>& indices_to_add, int &reference_cloud_flag){


    run_time_1 = clock();
    // read input 

    log_file_time << recover_frame << ",";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_add (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_recovered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ExtractIndices<pcl::PointXYZ> basemap_extract;
    if(initExtractObject(basemap_extract, basemap, false )){
        std::cout<<"\t"<<"basemap extract object created successfully" << std::endl;
    }


    std::string compressed_input= recover_day_path + "/" + input_folder + "/" 
                                            +result_folder+"/" + recover_frame + output_extension;

    std::string reference_frame_name ="";
    if(multiple_day_flag==0){
        reference_frame_name =  reference_day_path + "/" + pcds_folder_name + "/" + reference_frame +".pcd";
    }
    else {
        reference_frame_name =  reference_day_path + "/" + reference_frame +".pcd";
    }
  


    std::vector<std::string> pose_recover_frame = get_pose(recover_day_path+"/" + pose_file_name, recover_frame, seperator);
    Eigen::Matrix4f transformation_matrix_recover_frame = get_transformation_matrix(pose_recover_frame);

    std::vector<uint8_t> compressedData;
    if(!readBinaryFileToVector(compressed_input, compressedData)){
        std::cout<<"failed to read compressed file"<<std::endl;
        return false;
    }

    if(!readPCD(reference_frame_name, cloud_recovered)){
        std::cout<< "\t"<<"refernce frame read fail" << std::endl;
        return false;
    }

    if (filter){
        removeZeroPoints(cloud_recovered);
    }
    
    run_time_2 = clock();
    //disjoin compressed indices and pcd
    uint32_t size_compressed_pcd;
    std::vector<char> compressedPCD;
    std::vector<uint8_t> compressed_indices;

    bytes_to_uint32(compressedData, size_compressed_pcd);
    compressedPCD.insert(compressedPCD.end(), compressedData.begin()+4, compressedData.begin()+4+size_compressed_pcd);
    compressed_indices.insert(compressed_indices.end(), compressedData.begin()+4+size_compressed_pcd, compressedData.end());


    run_time_3 = clock();
    //decompress point cloud
    DracoDecodePointCloudFromFile(compressedPCD, cloud_add);


    run_time_4 = clock();
    //decompress indices
    std::vector<int> decompressed_indices;
        if(!DecompressVectorLzma(compressed_indices, decompressed_indices)){
        return false;
    }
    reference_cloud_flag = decompressed_indices.back();
    decompressed_indices.pop_back();
    reference_cloud_indices.insert(reference_cloud_indices.end(), decompressed_indices.begin()+1,decompressed_indices.begin()+1+ decompressed_indices[0]);
    indices_to_add.insert(indices_to_add.end(), decompressed_indices.begin()+1+ decompressed_indices[0],decompressed_indices.end());
    
    run_time_5 = clock();
    // recover from diff
    recoverFromDiffVector(reference_cloud_indices);
    recoverFromDiffVector(indices_to_add);

    run_time_6 = clock();
    // extract indices reference pcd
    if (reference_cloud_indices.size()>0){
        if (reference_cloud_flag == 0 ){
            extractPoints(cloud_recovered, cloud_recovered, reference_cloud_indices, true);
        }
        else{
            extractPoints(cloud_recovered, cloud_recovered, reference_cloud_indices, false);

        }
    }

    
    run_time_7 = clock();
    // extract indices basemap pcd
    if(indices_to_add.size()>0){
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        inliers->indices = indices_to_add;
        extractFilter(basemap_extract, cloud_base_recovered, inliers);
        pcl::transformPointCloud (*cloud_base_recovered, *cloud_base_recovered, transformation_matrix_recover_frame.inverse());
    }

    run_time_8 = clock();
    // merge clouds 
    *cloud_recovered += *cloud_add ;
    *cloud_recovered += *cloud_base_recovered;


    run_time_9 = clock();


    return true;

}

bool recover_frames (std::string recover_day_path, std::string reference_day_path, std::string basemap_path, std::string input_folder, std::string output_folder){


    std::string output_path = recover_day_path + "/" + output_folder  ;
    // std::string decompressed_indices_to_add = recover_day_path + "/" + input_folder + "/" +add_indices_sorted_decompressed_folder;
    // std::string decompressed_indices_to_remove = recover_day_path + "/" + input_folder + "/" +remove_indices_sorted_decompressed_folder;
    // std::string icp_transforms_path = recover_day_path + "/" + input_folder + "/" +icp_transforms_file;

    
    std::vector<std::string> direcotries_to_create; 

    direcotries_to_create.push_back(output_path);
    // direcotries_to_create.push_back(decompressed_indices_to_add);
    // direcotries_to_create.push_back(decompressed_indices_to_remove);

    float run_time_a, run_time_b;

    run_time_a = clock();
    std::cout << "creating output directories" << std::endl;
    if(!createDirectories(direcotries_to_create)){
        std::cout<< "fail to create directories " << std::endl; 
        return false;
    }
    std::cout << "\t"<<"output directories created successfully" << std::endl;
    run_time_b = clock();
    float creatining_directoies = (float)(run_time_b-run_time_a)/CLOCKS_PER_SEC;

    run_time_a = clock();
    std::cout << "reading basemap" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr basemap (new pcl::PointCloud<pcl::PointXYZ> );
    if(readPCD(basemap_path+"/" + basemap_pcd_name, basemap)){
        std::cout<< "\t"<<"basemap read successfully" << std::endl;
    }
    run_time_b = clock();
    float reading_basemap = (float)(run_time_b-run_time_a)/CLOCKS_PER_SEC;


    run_time_a = clock();
    // std::cout << "creating basemap extract object" << std::endl;
    // pcl::ExtractIndices<pcl::PointXYZ> basemap_extract;
    // if(initExtractObject(basemap_extract, basemap, false )){
    //     std::cout<<"\t"<<"basemap extract object created successfully" << std::endl;
    // }
    run_time_b = clock();
    float extracting_basemap = (float)(run_time_b-run_time_a)/CLOCKS_PER_SEC;


    run_time_a = clock();
    std::cout << "reading association file" << std::endl;
    std::vector<std::string> recover_frames ;
    std::vector<std::string> reference_frames ;
    std::vector<float> distances ;

    std::string line ;
    std::ifstream association_file(recover_day_path+"/association.txt");
    
    while (getline (association_file, line )) 
    {
        std::vector<std::string> split_line = splitString(line, ',');
        std::string compress_frame = split_line[0];
        std::string reference_frame = split_line[1];
        float distance = std::stof(split_line[2]);
        if (distance < distance_threshold){
            recover_frames.push_back(compress_frame);
            reference_frames.push_back(reference_frame);
            distances.push_back(distance);
        }
  
    }
    association_file.close();
    std::cout << "\t"<<"association file read successfully" << std::endl;
    run_time_b = clock();
    float read_association_file = (float)(run_time_b-run_time_a)/CLOCKS_PER_SEC;

    log_file_time.open(recover_day_path+"/log_file_time_decompression.csv");

    log_file_time << "creting directories" << "," << creatining_directoies <<std::endl;
    log_file_time << "reading_basemap" << "," << reading_basemap <<std::endl;
    log_file_time << "basemap_extract" << "," << extracting_basemap <<std::endl;
    log_file_time << "reading_association_file" << "," << read_association_file <<std::endl;

    log_file_time << "frame_number" << "," << "read" << "," << "disjoin" << "," <<"decompress_pcd" << "," <<"decompress_indices" << 
                    "," <<"recover_diff" <<","<< "extract_points_reference_cloud" << "," << "extract_points_basemap" << 
                    "," << "merge" << "," << "write" << ","<< "total" << std::endl;


    // #pragma omp parallel for
    for (int i=0; i<recover_frames.size(); i++){

        // creating variabless to store decompressed output
        std::vector<int> reference_cloud_indices;
        int reference_cloud_flag =0 ; // This flag tells wheahter the common points or exclusive points are stored for reference cloud. 
        std::vector<int> indices_to_add;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_recovered (new pcl::PointCloud<pcl::PointXYZ> );

        
        std::string recover_frame = recover_frames[i] ;
        std::string reference_frame = reference_frames[i];

        std::cout<< recover_frame<< std::endl;

        // running decompressoin algo for each point cloud.
        if (!recover (reference_day_path, reference_frame, recover_day_path, recover_frame, input_folder, 
            basemap, cloud_recovered,  reference_cloud_indices, indices_to_add,reference_cloud_flag)){
        }

        run_time_10 = clock();
        // writng output 
        pcl::io::savePCDFileBinary (output_path +"/"+ recover_frame +".pcd", *cloud_recovered);
        // writeVectorToFile(decompressed_indices_to_add+"/" + recover_frame + ".dat", indices_to_add);
        // writeVectorToFile(decompressed_indices_to_remove+"/" + recover_frame + ".dat", indices_to_remove);

        run_time_11 = clock();



        float read =                    (float)(run_time_2-run_time_1)/CLOCKS_PER_SEC;
        float disjoin =                 (float)(run_time_3-run_time_2)/CLOCKS_PER_SEC;
        float decompress_pcd =          (float)(run_time_4-run_time_3)/CLOCKS_PER_SEC;
        float decompress_indices =      (float)(run_time_5-run_time_4)/CLOCKS_PER_SEC;
        float recover_diff =            (float)(run_time_6-run_time_5)/CLOCKS_PER_SEC;
        float extract_points_ref =      (float)(run_time_7-run_time_6)/CLOCKS_PER_SEC;
        float extract_points_basemap =  (float)(run_time_8-run_time_7)/CLOCKS_PER_SEC;
        float merge =                   (float)(run_time_9-run_time_8)/CLOCKS_PER_SEC;
        float write =                   (float)(run_time_11-run_time_10)/CLOCKS_PER_SEC;
        float total =                   (float)(run_time_9-run_time_2)/CLOCKS_PER_SEC;


        log_file_time << read << "," << disjoin << "," << decompress_pcd << "," << decompress_indices << 
                        "," << recover_diff << "," << extract_points_ref << ","<< extract_points_basemap << 
                        "," << merge << "," << write << "," << total << std::endl;

    }
    
    return true;
}

int main (int argc, char** argv)
{
    std::string data_path = std::string(argv[1]);
    std::string basemap_path = data_path;


    std::string recover_day_path = data_path + "/" + std::string(argv[3]);
    std::string input_folder = std::string(argv[4]);
    std::string output_folder = std::string(argv[5]);

    multiple_day_flag = std::stof(argv[6]);

    std::string reference_day_path ="";

    if(multiple_day_flag==0){
        reference_day_path = data_path+ "/" +std::string(argv[2]);
    }
    else {
        reference_day_path = data_path; 
    }


    recover_frames (recover_day_path, reference_day_path, basemap_path, input_folder, output_folder);

  return (0);
}
