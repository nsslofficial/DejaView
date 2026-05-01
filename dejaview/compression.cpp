#include "my_lzma.h"
#include "my_pcl.h"
#include "my_utils.h"
#include "my_draco.h"
#include "my_variables.h"
#include <typeinfo>
#include <omp.h>
#include <chrono>
#include <set>  



std::ofstream log_file_time,log_file_size;

std::chrono::steady_clock::time_point run_time_1, run_time_2, run_time_3, run_time_4, run_time_5, run_time_6, run_time_7 ,run_time_8, 
         run_time_9,run_time_10, run_time_11 , run_time_12, run_time_13, run_time_14 ,run_time_15, run_time_16;


bool diffVector(std::vector<int>& vector){
    // convert vector to diff vector 
    for (int i = vector.size() - 1; i > 0; --i) {
        vector[i] = vector[i - 1] - vector[i];
    }    
    return true;
}

std::vector<int> vectorDifference(const std::vector<int>& v1, const std::vector<int>& v2) {
    std::vector<int> diff;
    
    // Use a set for fast lookup
    std::set<int> set_v2(v2.begin(), v2.end());

    // Add elements from v1 that are not in v2
    for (int elem : v1) {
        if (set_v2.find(elem) == set_v2.end()) {
            diff.push_back(elem);
        }
    }
    
    return diff;
}

bool CompressVectorLzma(std::vector<int>& vector, std::vector<uint8_t>& compressedData){
    
    // copy input int vector to a uint_8 vector
    std::vector<uint8_t> byteData(vector.size() * sizeof(int));
    memcpy(byteData.data(), vector.data(), byteData.size());

    // Compression
    if(!compressVector(byteData, compressedData, buffer_size, preset)){
        std::cout << "compression failed" <<std::endl;
        return false;
    }

    return true;
}


bool find_change (std::string reference_day_path, std::string reference_frame, std::string compress_day_path,  std::string compress_frame, std::string basemap_path, 
pcl::KdTreeFLANN<pcl::PointXYZ>& basemap_kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr change_add, std::vector<int>& reference_cloud_indices, std::vector<int>& indices_to_remove,
std::vector<int>& indices_to_add, int& reference_cloud_flag, int& count)
{

    run_time_1 = std::chrono::steady_clock::now();

    // read input 
    log_file_time << compress_frame << ",";
    log_file_size << compress_frame << ",";


    float resolutionSquare = resolution*resolution;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reference (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_compress (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transformation_matrix_compress_frame;


    std::string compress_frame_name = compress_day_path + "/" + pcds_folder_name + "/" + compress_frame + ".pcd";
    std::string reference_frame_name ="";
    if(multiple_day_flag==0){
        reference_frame_name =  reference_day_path + "/" + pcds_folder_name + "/" + reference_frame +".pcd";
    }
    else {
        reference_frame_name =  reference_day_path + "/" + reference_frame +".pcd";
    }

    
    if(!readPCD(compress_frame_name, cloud_compress)){
        std::cout<< "\t"<<"compress frame read fail." << std::endl;
    }
    

    if(!readPCD(reference_frame_name, cloud_reference)){
        std::cout<< "\t"<<"refernce frame read fail" << std::endl;
    }

    std::vector<std::string> pose_compress_frame= get_pose(compress_day_path+"/" +pose_file_name, compress_frame, seperator);


    run_time_2 = std::chrono::steady_clock::now();
    //filtering

    if (filter)
    {
        removeZeroPoints(cloud_reference);
        removeZeroPoints(cloud_compress);
    } 
    log_file_size << cloud_compress->size() << ",";

    run_time_3 = std::chrono::steady_clock::now();
    // coarse new points using octree 
    std::vector<int> coarse_indices_add;
    coarseChangeOctree(cloud_reference, cloud_compress, coarse_indices_add, resolution);
    // log_file_size << coarse_indices_add.size() << ",";


    run_time_4 = std::chrono::steady_clock::now();
    // // fine new points using kDTree
    std::vector<int> fine_indices_add;
    fineChangeKdtree(cloud_reference, cloud_compress, coarse_indices_add, fine_indices_add, NN_distance*NN_distance, K);
    extractPoints(cloud_compress, change_add, fine_indices_add, false );
    // log_file_size << fine_indices_add.size() << ",";

    run_time_5 = std::chrono::steady_clock::now();
    // redundant points from basemap using kdtree
    transformation_matrix_compress_frame = get_transformation_matrix(pose_compress_frame);
    pcl::transformPointCloud (*change_add, *change_add, transformation_matrix_compress_frame);
    std::vector<int> indices_to_remove_from_add;
    getKDtreeNearestNeighbourPoints(basemap_kdtree, change_add , indices_to_remove_from_add, indices_to_add, NN_distance*NN_distance, K);
    
    // extract points from cloud 
    extractPoints(change_add, change_add, indices_to_remove_from_add, true );
    pcl::transformPointCloud (*change_add, *change_add, transformation_matrix_compress_frame.inverse());
    // log_file_size << change_add->size() << ",";


    run_time_6 = std::chrono::steady_clock::now();
    // coarse points to remove using Octree 
    // log_file_size << cloud_reference->size() << ",";
    std::vector<int> coarse_indices_remove;
    coarseChangeOctree(cloud_compress, cloud_reference, coarse_indices_remove, resolution);
    // log_file_size << coarse_indices_remove.size() << ",";

    std::vector<int> fine_indices_remove;

    run_time_7 = std::chrono::steady_clock::now();
    //  fine points to remove using KDTree 
    fineChangeKdtree(cloud_compress,cloud_reference,  coarse_indices_remove, fine_indices_remove, NN_distance*NN_distance, K);

    // std::cout << "new points: "<< fine_indices_remove.size() << std::endl;
    // std::cout << "cloud points: "<< cloud_reference->size() << std::endl;

    if (fine_indices_remove.size() > cloud_reference->size()/2){
        reference_cloud_flag=1; 
        count++;
        std::vector<int> all_indices_reference(cloud_reference->size()); 
        std::iota(all_indices_reference.begin(), all_indices_reference.end(), 0);

        reference_cloud_indices = vectorDifference(all_indices_reference, fine_indices_remove);
    }
    else{
        reference_cloud_indices = fine_indices_remove;
    }
    indices_to_remove = fine_indices_remove;
    // std::cout << "savedd points: "<< reference_cloud_indices.size() << std::endl;
    // std::cout << "count: " << count <<  std::endl;

    // log_file_size << indices_to_remove.size() << std::endl;
    // std::cout<<reference_cloud_indices.size() <<std::endl;
    run_time_8 = std::chrono::steady_clock::now();

    return true;
}



bool compress_frames (std::string compress_day_path, std::string reference_day_path,  std::string basemap_path, std::string output_folder)
{

    // std::string pcd_subset_folder =                     compress_day_path + "/" + subset_pcd_folder ;
    std::string output_path =                           compress_day_path + "/" + output_folder;
    // std::string add_results_folder_pcd =                output_path+ "/" + add_pcd_folder ;
    // std::string add_results_folder_compressed_pcd =     output_path+ "/" + add_pcd_folder_compressed ;
    // std::string add_results_folder_ply =                output_path+ "/" + add_ply_folder ;
    // std::string add_indices_results_folder =            output_path+ "/" + add_indices_folder  ;
    // std::string add_indices_results_folder_sorted =     output_path+ "/" + add_indices_folder_sorted ;
    // std::string remove_results_folder =                 output_path+ "/" + remove_indices_folder ;
    // std::string remove_results_folder_sorted =          output_path+ "/" + remove_indices_folder_sorted ;
    // std::string remove_results_folder_compressed =      output_path+ "/" + remove_indices_folder_compressed ;
    // std::string add_indices_results_folder_compressed = output_path+ "/" + add_indices_folder_compressed ;
    // std::string indices_result_folder_compressed =      output_path+ "/" + indices_folder_compressed ;
    std::string result_folder_compressed =      output_path+ "/" + result_folder ;



    std::vector<std::string> direcotries_to_create; 

    // direcotries_to_create.push_back(pcd_subset_folder);
    // direcotries_to_create.push_back(add_results_folder_pcd);
    // direcotries_to_create.push_back(add_results_folder_ply);
    // direcotries_to_create.push_back(add_indices_results_folder);
    // direcotries_to_create.push_back(add_indices_results_folder_sorted);
    // direcotries_to_create.push_back(remove_results_folder);
    // direcotries_to_create.push_back(remove_results_folder_sorted);
    // direcotries_to_create.push_back(remove_results_folder_compressed);
    // direcotries_to_create.push_back(add_indices_results_folder_compressed);
    // direcotries_to_create.push_back(add_results_folder_compressed_pcd);
    // direcotries_to_create.push_back(indices_result_folder_compressed);
    direcotries_to_create.push_back(result_folder_compressed);

    float run_time_a, run_time_b ; 

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
    std::cout << "creating basemap kdtree object" << std::endl;
    pcl::KdTreeFLANN<pcl::PointXYZ> basemap_kdtree;
    if(initKDtreeObject(basemap_kdtree, basemap)){
        std::cout<<"\t"<<"basemap kdtree object created successfully" << std::endl;
    }
    run_time_b = clock();
    float basemap_kdtree_time = (float)(run_time_b-run_time_a)/CLOCKS_PER_SEC;

    // run_time_a = clock();
    // std::cout << "creating basemap octree search object" << std::endl;
    // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> basemap_octree(resolution);
    // if(initOctreeSearchObject(basemap_octree, basemap)){
    //     std::cout<<"\t"<<"basemap octree search object created successfully" << std::endl;
    // }
    // run_time_b = clock();
    // float basemap_octree_time = (float)(run_time_b-run_time_a)/CLOCKS_PER_SEC;



    // run_time_a = clock();
    // std::cout << "creating basemap extract object" << std::endl;
    // pcl::ExtractIndices<pcl::PointXYZ> basemap_extract;
    // if(initExtractObject(basemap_extract, basemap, false )){
    //     std::cout<<"\t"<<"basemap extract object created successfully" << std::endl;
    // }
    // run_time_b = clock();
    // float extracting_basemap = (float)(run_time_b-run_time_a)/CLOCKS_PER_SEC;


    
    run_time_a = clock();
    std::cout << "reading association file" << std::endl;
    std::vector<std::string> compress_frames ;
    std::vector<std::string> reference_frames ;
    std::vector<float> distances ;

    std::string line ;
    std::ifstream association_file(compress_day_path+"/association.txt");
    
    while (getline (association_file, line )) 
    {
        std::vector<std::string> split_line = splitString(line, ',');
        std::string compress_frame = split_line[0];
        std::string reference_frame = split_line[1];
        float distance = std::stof(split_line[2]);
        if (distance < distance_threshold){
            compress_frames.push_back(compress_frame);
            reference_frames.push_back(reference_frame);
            distances.push_back(distance);
        }
  
    }
    association_file.close();
    std::cout << "\t"<<"association file read successfully" << std::endl;
    run_time_b = clock();
    float read_association_file = (float)(run_time_b-run_time_a)/CLOCKS_PER_SEC;

      

    log_file_time.open(output_path+ "/" +"/log_file_time_compression.csv");

    log_file_time << "creting directories" << "," << creatining_directoies <<std::endl;
    log_file_time << "reading_basemap" << "," << reading_basemap <<std::endl;
    log_file_time << "basemap_kdtree" << "," << basemap_kdtree_time <<std::endl;
    // log_file_time << "basemap_octree" << "," << basemap_octree_time <<std::endl;
    // log_file_time << "basemap_extract" << "," << extracting_basemap <<std::endl;
    log_file_time << "reading_association_file" << "," << read_association_file <<std::endl;


    log_file_time << "frame_number" << "," << "read" << "," << "filter" << ","<<"N_points(octree)" << 
                    "," << "N_points(kdtree)" << "," << "N_points_basemap(kdtree)"<< 
                    "," << "R_points(octree)" << "," << "R_points(kdtree)" <<  
                    "," << "draco compression" << ","  << "compress_indices" << "," << "concatenate output" <<
                    "," << "write" << "," << "total" << std::endl;


    log_file_size.open(output_path+ "/" +"/log_file_size_compression.csv");
    log_file_size << "compress_frame" << "," << "original_size"<< ","<<"N_points(octree)" << "," << "N_points(kdtree)" << "," 
            << "N_points_basemap(kdtree)"<< ","<< "reference_frame_size" << ","<< "R_points(octree)" << "," << "R_points(kdtree)" << std::endl;


    int count =0 ; // This flag tells wheahter the common points or exclusive points are stored for reference cloud. 
    int diff_in_size= 0;
    int diff_in_size_compression= 0;
    // runnig compression algo for each frame
    // #pragma omp parallel for
    for (int i=0; i<compress_frames.size(); i++){
        
        

        // initializing variables to store lean representation.
        std::vector<int> reference_cloud_indices;
        int reference_cloud_flag =0 ; // This flag tells wheahter the common points or exclusive points are stored for reference cloud. 
        std::vector<int> indices_to_add;
        std::vector<int> indices_to_remove;



        pcl::PointCloud<pcl::PointXYZ>::Ptr change_add (new pcl::PointCloud<pcl::PointXYZ> );

        std::string compress_frame = compress_frames[i] ;
        std::string reference_frame = reference_frames[i];
        // float distance = distances[i];

        std::cout <<compress_frame<<std::endl;

        // calling the func that find lean representation and saving the reults.
        if (find_change (reference_day_path, reference_frame, compress_day_path, compress_frame, basemap_path, basemap_kdtree,
          change_add, reference_cloud_indices, indices_to_remove, indices_to_add, reference_cloud_flag, count)){ 
            
            run_time_9 = std::chrono::steady_clock::now(); 

            // variables to store comporessed output 
            std::vector<uint8_t> compressedPCD;
            std::vector<int> concatenated_indices;
            std::vector<int> concatenated_indices_2;
            std::vector<uint8_t> compressed_concatenated_indices_2;
            std::vector<uint8_t> compressed_concatenated_indices;
            std::vector<uint8_t> compressed_output;
            std::vector<uint8_t> compressed_add, compressed_remove;



  
            
            // draco compression
            if(change_add->size() > 0){
                DracoEncodePointCloudToFile(change_add, "", draco_quantization_parameter, draco_compression_level, compressedPCD);
            }

            run_time_10 = std::chrono::steady_clock::now(); 

            // sort and diff vector 
            if (reference_cloud_indices.size() >0){
                std::sort(reference_cloud_indices.begin(), reference_cloud_indices.end(), std::greater<int>());
                if (reference_cloud_flag == 0){
                    reference_cloud_indices.erase(std::unique(reference_cloud_indices.begin(), reference_cloud_indices.end()), reference_cloud_indices.end());
                }
                diffVector(reference_cloud_indices);
            }

             // sort and diff vector 
            if (indices_to_remove.size() >0){
                std::sort(indices_to_remove.begin(), indices_to_remove.end(), std::greater<int>());
                indices_to_remove.erase(std::unique(indices_to_remove.begin(), indices_to_remove.end()), indices_to_remove.end());
                diffVector(indices_to_remove);
            }
    

        
            // sort and diff vector 
            if (indices_to_add.size() >0){
                std::sort(indices_to_add.begin(), indices_to_add.end(), std::greater<int>());
                indices_to_add.erase(std::unique(indices_to_add.begin(), indices_to_add.end()), indices_to_add.end());
                diffVector(indices_to_add);
            }
    

    
            // compress vector
            concatenated_indices.push_back(reference_cloud_indices.size());
            concatenated_indices.insert(concatenated_indices.end(), reference_cloud_indices.begin(), reference_cloud_indices.end());
            concatenated_indices.insert(concatenated_indices.end(), indices_to_add.begin(), indices_to_add.end());
            concatenated_indices.push_back(reference_cloud_flag);
            CompressVectorLzma(concatenated_indices, compressed_concatenated_indices);

            // // concatenated_indices_2.push_back(indices_to_remove.size());
            // concatenated_indices_2.insert(concatenated_indices_2.end(), indices_to_remove.begin(), indices_to_remove.end());
            // concatenated_indices_2.insert(concatenated_indices_2.end(), indices_to_add.begin(), indices_to_add.end());
            // CompressVectorLzma(concatenated_indices_2, compressed_concatenated_indices_2);

            // std::cout << "concatenated indices size:  " << concatenated_indices.size() <<std::endl;
            // std::cout << "concatenated indices 2 size:  " << concatenated_indices_2.size() <<std::endl;

            // diff_in_size =  (concatenated_indices_2.size() - concatenated_indices.size());
            // diff_in_size_compression = (compressed_concatenated_indices_2.size() - compressed_concatenated_indices.size());


            // std::cout << "diff in size before compression :  " << diff_in_size <<std::endl;
            // std::cout << "diff in size after compression :  " << diff_in_size_compression <<std::endl;
 
            run_time_11 = std::chrono::steady_clock::now(); 
            //concatenate compress indices and pcd
            uint32_t size_compressed_pcd = compressedPCD.size();
            std::vector<uint8_t> size_compressed_pcd_bytes(4);
            uint32_to_bytes(size_compressed_pcd, size_compressed_pcd_bytes);
            compressed_output.insert(compressed_output.end(), size_compressed_pcd_bytes.begin(), size_compressed_pcd_bytes.end());
            compressed_output.insert(compressed_output.end(), compressedPCD.begin(), compressedPCD.end());
            compressed_output.insert(compressed_output.end(), compressed_concatenated_indices.begin(), compressed_concatenated_indices.end());

            run_time_12 = std::chrono::steady_clock::now(); 
            writeVectorToBinaryFile(result_folder_compressed+"/"+ compress_frame +output_extension, compressed_output);
            
            run_time_13 = std::chrono::steady_clock::now();

        }


        auto read =                std::chrono::duration_cast<std::chrono::milliseconds>(run_time_2-run_time_1);
        auto filter =              std::chrono::duration_cast<std::chrono::milliseconds>(run_time_3-run_time_2);
        auto new_points_octree =   std::chrono::duration_cast<std::chrono::milliseconds>(run_time_4-run_time_3);
        auto new_points_kdtree =   std::chrono::duration_cast<std::chrono::milliseconds>(run_time_5-run_time_4);
        auto new_points_basemap =  std::chrono::duration_cast<std::chrono::milliseconds>(run_time_6-run_time_5);
        auto R_points_octree =     std::chrono::duration_cast<std::chrono::milliseconds>(run_time_7-run_time_6);
        auto R_points_kdtree =     std::chrono::duration_cast<std::chrono::milliseconds>(run_time_8-run_time_7);
        auto draco_compression =   std::chrono::duration_cast<std::chrono::milliseconds>(run_time_10-run_time_9);
        auto compress_indices =    std::chrono::duration_cast<std::chrono::milliseconds>(run_time_11-run_time_10);
        auto concatencate_output = std::chrono::duration_cast<std::chrono::milliseconds>(run_time_12-run_time_11);
        auto write =               std::chrono::duration_cast<std::chrono::milliseconds>(run_time_13-run_time_12);
        auto total =               std::chrono::duration_cast<std::chrono::milliseconds>(run_time_12-run_time_3);


        log_file_time << read.count() << "," << filter.count() << "," << new_points_octree.count() << ","<< new_points_kdtree.count() << "," << new_points_basemap.count() << 
                                "," << R_points_octree.count()<< "," << R_points_kdtree.count() << 
                                "," << draco_compression.count() << "," <<  compress_indices.count() << "," << concatencate_output.count() << 
                                "," << write.count() << "," << total.count()<< std::endl;
        // log_file_time <<  total.count()<< std::endl;
    }
    
    
    return true;
}

int main(int argc, char** argv) {

    std::string dataset_folder = std::string(argv[1]);
    std::string basemap_path = dataset_folder;

    std::string compress_day_path = dataset_folder + "/" + std::string(argv[3]);
    std::string output_folder = std::string(argv[4]);

    resolution = std::stof(argv[5]) ;
    NN_distance = std::stof(argv[5]);

    multiple_day_flag = std::stof(argv[6]);

    std::string reference_day_path ="";

    if(multiple_day_flag==0){
        reference_day_path = dataset_folder+ "/" +std::string(argv[2]);
    }
    else {
        reference_day_path = dataset_folder; 
    }

    compress_frames(compress_day_path, reference_day_path,basemap_path, output_folder);

    return 0;
}