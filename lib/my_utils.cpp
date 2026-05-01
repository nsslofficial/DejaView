#include "my_utils.h"

std::vector<std::string> splitString(std::string str, char splitter){
    std::vector<std::string> result;
    std::string current = ""; 
    for(int i = 0; i < str.size(); i++){
        if(str[i] == splitter){
            if(current != ""){
                result.push_back(current);
                current = "";
            } 
            continue;
        }
        current += str[i];
    }
    if(current.size() != 0)
        result.push_back(current);
    return result;
}

std::vector<float> splitStringFloat(std::string str, char splitter){
    std::vector<float> result;
    std::string current = ""; 
    for(int i = 0; i < str.size(); i++){
        if(str[i] == splitter){
            if(current != ""){
                result.push_back(std::stof(current));
                current = "";
            } 
            continue;
        }
        current += str[i];
    }
    if(current.size() != 0)
        result.push_back(std::stof(current));
    return result;
}

bool createDirectory(std::string path){
    if (!std::filesystem::is_directory(path) || !std::filesystem::exists(path)) 
    { 
        std::filesystem::create_directories(path);
    }
    return true;
}


bool createDirectories(std::vector<std::string> path_vector){
    for (const std::string& path : path_vector) {
        if(!createDirectory(path)){
            std::cout<< "fail to create directory for "<< path << std::endl; 
            return false;
        }
    }
    return true;
}

Eigen::Matrix4f convert_YPR_to_rotation_matrix(float yaw, float pitch, float roll)
{
    Eigen::Matrix4f rotaiton_matrix = Eigen::Matrix4f::Identity();
    yaw = (M_PI *yaw)/180.0 ;
    pitch = (M_PI *pitch)/180.0 ;
    roll = (M_PI *roll)/180.0 ;
    rotaiton_matrix (0,0) = std::cos(yaw) * std::cos(pitch);
    rotaiton_matrix (1,0) = sin(yaw) * std::cos(pitch);
    rotaiton_matrix (2,0) = -sin(pitch);


    rotaiton_matrix (0,1) = std::cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*std::cos(roll);
    rotaiton_matrix (1,1) = sin(yaw)*sin(pitch)*sin(roll) + std::cos(yaw)*std::cos(roll);
    rotaiton_matrix (2,1) = std::cos(pitch)*sin(roll);

    rotaiton_matrix (0,2) = std::cos(yaw)*sin(pitch)*std::cos(roll) + sin(yaw)*sin(roll);
    rotaiton_matrix (1,2) = sin(yaw)*sin(pitch)*std::cos(roll) - std::cos(yaw)*sin(roll);
    rotaiton_matrix (2,2) = std::cos(pitch)*std::cos(roll);

    return rotaiton_matrix;

}

Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> pose )
{
    Eigen::Matrix4f rotaiton_matrix = convert_YPR_to_rotation_matrix(std::stof(pose[4]),std::stof(pose[5]),std::stof(pose[6]));
    rotaiton_matrix (0,3) = std::stof(pose[1]);
    rotaiton_matrix (1,3) = std::stof(pose[2]);
    rotaiton_matrix (2,3) = std::stof(pose[3]);

    return rotaiton_matrix;
}

std::vector<std::string> get_pose(std::string path, std::string frame_number, char splitter){
    
    std::ifstream pose_file(path);
    std::string line;

    while(getline (pose_file, line)) 
    {   
        std::vector<std::string> split_line = splitString(line, splitter);
        if (frame_number == split_line[0]) 
            return split_line;
    }

}

bool readTransformFromFile(std::string path, std::vector<float>& pose) {

  std::ifstream pose_file(path);
  std::string line;

  while(getline (pose_file, line)){
    std::vector<float> split_line = splitStringFloat(line, ',');
    pose.insert(std::end(pose), std::begin(split_line), std::end(split_line));
  }
  return true;
}

bool writeVectorToFile(std::string filename, std::vector<int> data){
    std::ofstream outfile(filename, std::ios::out | std::ofstream::binary);
    if (!outfile) {
        std::cerr << "Error opening file!" << std::endl;
        return false;
    }
    std::ostream_iterator<int> out_itr(outfile, "\n"); 
    std::copy(data.begin(), data.end(),out_itr );
    outfile.close();
    return true;
}

bool writeVectorToBinaryFile(std::string filename, std::vector<uint8_t> data){
    std::ofstream outFile(filename, std::ios::binary);
    if (!outFile) {
        std::cerr << "Error opening file!" << std::endl;
        return false;
    }
    outFile.write(reinterpret_cast<const char*>(data.data()), data.size());
    outFile.close();
    return true;
}

bool readBinaryFileToVector(std::string filename, std::vector<uint8_t>& data){

    std::ifstream inFile(filename, std::ios::binary);
    if (!inFile) {
        std::cerr << "Error opening file!" << std::endl;
        return false;
    }
    // Determine the size of the file
    inFile.seekg(0, std::ios::end);
    std::streampos fileSize = inFile.tellg();
    inFile.seekg(0, std::ios::beg);

    // Create a vector to hold the data
    data.resize(fileSize);

    // Read the data from the file into the vector
    inFile.read(reinterpret_cast<char*>(data.data()), fileSize);


    // Close the file
    inFile.close();

    return true;
}


bool uint32_to_bytes(uint32_t value, std::vector<uint8_t>& bytes) {
   
    bytes[0] = value & 0xFF;         // Least significant byte
    bytes[1] = (value >> 8) & 0xFF;
    bytes[2] = (value >> 16) & 0xFF;
    bytes[3] = (value >> 24) & 0xFF; // Most significant byte

    return true;
}


bool bytes_to_uint32(std::vector<uint8_t>& bytes, uint32_t& value) {
    
    if (bytes.size() < 4) {
        // Handle error: not enough bytes to represent a uint32_t
        return false; // Return a default value or throw an exception
    }

    value = 0;

    value |= bytes[0];                  // Least significant byte
    value |= (static_cast<uint32_t>(bytes[1]) << 8);
    value |= (static_cast<uint32_t>(bytes[2]) << 16);
    value |= (static_cast<uint32_t>(bytes[3]) << 24); // Most significant byte

    return true;
}



bool stringToBool(const std::string& str) {
    // Convert the string to lowercase
    std::string lowerStr = str;
    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(), ::tolower);

    // Check for true values
    if (lowerStr == "true" || lowerStr == "1") {
        return true;
    }
    // Check for false values
    else if (lowerStr == "false" || lowerStr == "0") {
        return false;
    } else {
        throw std::invalid_argument("Invalid boolean value: " + str);
    }
}

bool writeStringStreamToBinaryFile( std::string file_path, std::stringstream& data){
    std::ofstream outFile(file_path, std::ios::binary);
    outFile << data.rdbuf();
    outFile.close();
    return true;
}

bool readStringStreamToBinaryFile(std::string file_path, std::stringstream& data){
    std::ifstream inFile(file_path,  std::ios::binary);
    data << inFile.rdbuf();
    inFile.close();
    return true;
}

bool printVector(std::vector<int> v){
    for (int i =0 ; i< v.size() ; i++){
        std::cout << v[i] <<std::endl;
    }
}
