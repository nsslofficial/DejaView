#ifndef MY_LZMA_H
#define MY_LZMA_H

#include <vector>
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <lzma.h>
#include <vector>
#include <random>


bool compressLZMA(const std::string& inFile, const std::string& outFile, int buffer_size, int preset);
bool decompressLZMA(const std::string& inFile, const std::string& outFile, int buffer_size, int preset);
bool compressVector(std::vector<uint8_t>& inputData, std::vector<uint8_t>& outputData, int buffer_size, int preset);
bool decompressVector(std::vector<uint8_t>& inputData, std::vector<uint8_t>& outputData, int buffer_size, int preset);
std::vector<int> generateRandomVector(int size, int min, int max);
bool testVectorCompression(int buffer_size, int preset);
bool testFileCompression(std::string input_path, std::string compressed_path, std::string decompressed_path, int buffer_size, int preset);

#endif
