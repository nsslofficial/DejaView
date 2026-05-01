#ifndef MY_UTILS_H
#define MY_UTILS_H

#include <filesystem>
#include <vector>  
#include <string.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <iterator>
#include <algorithm> 
#include <sstream>
#include <chrono>


std::vector<std::string> splitString(std::string str, char splitter);
bool createDirectory(std::string path);
bool createDirectories(std::vector<std::string> path_vector);
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float yaw, float pitch, float roll);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> pose );
std::vector<std::string> get_pose(std::string path, std::string frame_number,char splitter);
bool readTransformFromFile(std::string path, std::vector<float>& pose);
bool writeVectorToFile(std::string filename, std::vector<int> data);
bool writeVectorToBinaryFile(std::string filename, std::vector<uint8_t> data);
bool readBinaryFileToVector(std::string filename, std::vector<uint8_t>& data);
bool uint32_to_bytes(uint32_t value, std::vector<uint8_t>& bytes);
bool bytes_to_uint32(std::vector<uint8_t>& bytes, uint32_t& value);
bool stringToBool(const std::string& str);
bool writeStringStreamToBinaryFile( std::string file_path, std::stringstream& data);
bool readStringStreamToBinaryFile( std::string file_path, std::stringstream& data);
bool printVector(std::vector<int> v);


#endif