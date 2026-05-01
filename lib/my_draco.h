
#ifndef MY_DRACO_H
#define MY_DRACO_H

#include <draco/io/file_utils.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/compression/encode.h>
#include <draco/compression/expert_encode.h>
#include <draco/compression/decode.h>
#include <pcl/io/pcd_io.h>



bool convertPCLtoDraco( pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::unique_ptr<draco::PointCloud>& draco_cloud);
bool convertDracoToPCL(std::unique_ptr<draco::PointCloud>& draco_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud );
bool initDracoEncoder(draco::Encoder& encoder, int quantization_bits, int compression_level);
bool DracoEncodePointCloudToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, const std::string& output_path,  int quantization_bits, int compression_level,  std::vector<uint8_t>& compressedData);
bool DracoDecodePointCloudFromFile( std::vector<char>& data, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);

#endif