#include "my_lzma.h"


std::vector<int> generateRandomVector(int size, int min, int max) {
    std::vector<int> randomVector;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(min, max);

    for (int i = 0; i < size; ++i) {
        randomVector.push_back(dis(gen));
    }

    return randomVector;
}


bool compressLZMA(const std::string& inFile, const std::string& outFile, int buffer_size, int preset) {

    std::ifstream inputFile(inFile, std::ios::binary);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Couldn't open input file" << std::endl;
        return false;
    }

    std::ofstream outputFile(outFile, std::ios::binary);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Couldn't open output file" << std::endl;
        return false;
    }

    lzma_stream stream = LZMA_STREAM_INIT;

    // default preset value is 6 and can be accessed using LZMA_PRESET_DEFAULT
    // preset can have any value between 0 to 9 
    // In addition to being more CPU-intensive, compression with higher presets also requires 
    // much more memory (and produces output that needs more memory to decompress)
    if (lzma_easy_encoder(&stream, preset, LZMA_CHECK_CRC64) != LZMA_OK) {
        std::cerr << "Error initializing LZMA encoder" << std::endl;
        return false;
    }

    std::vector<uint8_t> inBuf(buffer_size); // Buffer for input data
    std::vector<uint8_t> outBuf(buffer_size); // Buffer for output data

    stream.next_out = outBuf.data();
    stream.avail_out = outBuf.size();

    bool success = true;
    lzma_action action = LZMA_RUN; // Initialize action to LZMA_RUN

    lzma_ret ret = lzma_code(&stream, action);
        
    while (true) {

    
        if (stream.avail_in == 0 ){
        inputFile.read(reinterpret_cast<char*>(inBuf.data()), inBuf.size());
        stream.next_in = inBuf.data();
        stream.avail_in = inputFile.gcount();
        }

        // change action for last chunk of data
        if (inputFile.eof()) {
            action = LZMA_FINISH; // If processing the last chunk of data, set action to LZMA_FINISH
        } else {
            action = LZMA_RUN; // Otherwise, continue compression with LZMA_RUN
        }

        ret = lzma_code(&stream, action);

        if (ret == LZMA_OK) {
            outputFile.write(reinterpret_cast<const char*>(outBuf.data()), (outBuf.size() - stream.avail_out));
            stream.next_out = outBuf.data();
            stream.avail_out = outBuf.size();
        }

        if (ret == LZMA_STREAM_END){
            outputFile.write(reinterpret_cast<const char*>(outBuf.data()), (outBuf.size() - stream.avail_out));
            break;
        } 

        if (ret != LZMA_OK) {
            std::cerr << "Error compressing data" << std::endl;
            success = false;
            break;
        }

    }

    lzma_end(&stream);
    inputFile.close();
    outputFile.close();
    return success;
}


bool decompressLZMA(const std::string& inFile, const std::string& outFile, int buffer_size, int preset) {

    std::ifstream inputFile(inFile, std::ios::binary);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Couldn't open input file" << std::endl;
        return false;
    }

    std::ofstream outputFile(outFile, std::ios::binary);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Couldn't open output file" << std::endl;
        return false;
    }

    lzma_stream stream = LZMA_STREAM_INIT;

    if (lzma_stream_decoder(&stream, UINT64_MAX, LZMA_CONCATENATED) != LZMA_OK) {
        std::cerr << "Error initializing LZMA decoder" << std::endl;
        return false;
    }

    std::vector<uint8_t> inBuf(buffer_size); // Buffer for input data
    std::vector<uint8_t> outBuf(buffer_size); // Buffer for output data

    stream.next_out = outBuf.data();
    stream.avail_out = outBuf.size();

    bool success = true;

    // Initialize action to LZMA_RUN
    lzma_action action = LZMA_RUN; 

    while (true) {

        if (stream.avail_in == 0){
        inputFile.read(reinterpret_cast<char*>(inBuf.data()), inBuf.size());
        stream.next_in = inBuf.data();
        stream.avail_in = inputFile.gcount();
        }

        // change action for last chunk of data
        if (inputFile.eof()) {
            action = LZMA_FINISH; // If processing the last chunk of data, set action to LZMA_FINISH
        } else {
            action = LZMA_RUN; // Otherwise, continue compression with LZMA_RUN
        }

        lzma_ret ret = lzma_code(&stream, action);

        if (ret == LZMA_OK) {
            outputFile.write(reinterpret_cast<const char*>(outBuf.data()), (outBuf.size() - stream.avail_out));
            stream.next_out = outBuf.data();
            stream.avail_out = outBuf.size();
        }

        if (ret == LZMA_STREAM_END){
            // std::cout << "inisde END" << std::endl;
            outputFile.write(reinterpret_cast<const char*>(outBuf.data()), (outBuf.size() - stream.avail_out));
            break;
        } 

        if (ret!= LZMA_OK) {
            std::cerr << "Error decompressing data" << std::endl;
            success = false;
            break;
        }

    }

    lzma_end(&stream);
    inputFile.close();
    outputFile.close();

    return success;
}

bool compressVector(std::vector<uint8_t>& inputData, std::vector<uint8_t>& outputData, int buffer_size, int preset) {

    // initiallize lzma stream
    lzma_stream stream = LZMA_STREAM_INIT;

    // default preset value is 6 and can be accessed using LZMA_PRESET_DEFAULT
    // preset can have any value between 0 to 9 
    // In addition to being more CPU-intensive, compression with higher presets also requires 
    // much more memory (and produces output that needs more memory to decompress)
    if (lzma_easy_encoder(&stream, preset, LZMA_CHECK_CRC64) != LZMA_OK) {
        std::cerr << "Error initializing LZMA encoder" << std::endl;
        return false;
    }

    // initialize buffer for input and output data. 
    std::vector<uint8_t> inBuf(buffer_size); 
    std::vector<uint8_t> outBuf(buffer_size); 

    stream.next_out = outBuf.data();
    stream.avail_out = outBuf.size();

    // Initialize action to LZMA_RUN
    // the action changes to LZMA_FINISH when reading last chunk of data.
    lzma_action action = LZMA_RUN; 

    bool success = true;

    while (true) {

        // only read new data once previously read data is consumed
        if (stream.avail_in == 0 ){
            stream.next_in = &inputData[stream.total_in];
            stream.avail_in = std::min(inBuf.size(), inputData.size() - stream.total_in);
        }

        // change action for last chunk of data
        if (stream.total_in == inputData.size()) {
            action = LZMA_FINISH; // If processing the last chunk of data, set action to LZMA_FINISH
        } else {
            action = LZMA_RUN; // Otherwise, continue compression with LZMA_RUN
        }

        // run LZMA compression
        lzma_ret ret = lzma_code(&stream, action);

        if (ret == LZMA_OK) {
            outputData.insert(outputData.end(), outBuf.begin(), outBuf.begin() + (outBuf.size() - stream.avail_out));
            stream.next_out = outBuf.data();
            stream.avail_out = outBuf.size();
        }

        if (ret == LZMA_STREAM_END) {
            outputData.insert(outputData.end(), outBuf.begin(), outBuf.begin() + (outBuf.size() - stream.avail_out));
            break;
        } 
    
         if (ret != LZMA_OK) {
            std::cerr << "Error compressing data" << std::endl;
            success = false;
            break;
        }
        
    }

    lzma_end(&stream);

    return success;
}


bool decompressVector(std::vector<uint8_t>& inputData, std::vector<uint8_t>& outputData, int buffer_size, int preset) {

    // initiallize lzma stream
    lzma_stream stream = LZMA_STREAM_INIT;


    // preset value is not used for decompression
   if (lzma_stream_decoder(&stream, UINT64_MAX, LZMA_CONCATENATED) != LZMA_OK) {
        std::cerr << "Error initializing LZMA decoder" << std::endl;
        return false;
    }
    
    // initialize buffer for input and output data. 
    std::vector<uint8_t> inBuf(buffer_size); 
    std::vector<uint8_t> outBuf(buffer_size);

    stream.next_out = outBuf.data();
    stream.avail_out = outBuf.size();

    // Initialize action to LZMA_RUN
    // the action changes to LZMA_FINISH when reading last chunk of data.

    lzma_action action = LZMA_RUN;

    bool success = true;

    while (true) {

        // only read new data once previously read data is consumed
        if (stream.avail_in == 0 ){
        stream.next_in = &inputData[stream.total_in];
        stream.avail_in = std::min(inBuf.size(), inputData.size() - stream.total_in);
        }
        
        // change action for last chunk of data
        if (stream.total_in == inputData.size()) {
            action = LZMA_FINISH; // If processing the last chunk of data, set action to LZMA_FINISH
        } else {
            action = LZMA_RUN; // Otherwise, continue compression with LZMA_RUN 
        }

        // run LZMA decompression
        lzma_ret ret = lzma_code(&stream, action);

        if (ret == LZMA_OK ) {
            outputData.insert(outputData.end(), outBuf.data(), outBuf.data() + (outBuf.size() - stream.avail_out) );
            stream.next_out = outBuf.data();
            stream.avail_out = outBuf.size();
        }

        if (ret == LZMA_STREAM_END) {
            outputData.insert(outputData.end(), outBuf.data(), outBuf.data() + (outBuf.size() - stream.avail_out) );
            break;
        }

        if (ret != LZMA_OK) {
            std::cerr << "Error decompressing data" << std::endl;
            success = false;
            break;
        }
    }

    lzma_end(&stream);

    return success;
}


bool testVectorCompression(int buffer_size, int preset) {

    // generate random vector
    int size = 1000; // Size of the vector
    int min = 0;   // Minimum value
    int max = 255; // Maximum value
    std::vector<int> data = generateRandomVector(size, min, max);


    // copy input int vector to a uint_8 vector
    std::vector<uint8_t> byteData(data.size() * sizeof(int));
    memcpy(byteData.data(), data.data(), byteData.size());

    // Compression
    std::vector<uint8_t> compressedData;
    if(!compressVector(byteData, compressedData, buffer_size, preset)){
        std::cout << "compression failed" <<std::endl;
    }

    // Decompression
    std::vector<uint8_t> decompressedData;
    if(!decompressVector(compressedData,decompressedData, buffer_size, preset )){
        std::cout << "Decompression failed" <<std::endl;
    }

    // convert unit8 vector to int vector
    std::vector<int> output(decompressedData.size() / sizeof(int));
    memcpy(output.data(), decompressedData.data(), decompressedData.size());


    // check if decompressed vector is equal to original vector
    bool success = true ;
    if (output.size() == data.size()) {
        for (int i=0 ; i< output.size(); ++i){
            if (output[i] != data[i]){
                success =false;
            }
        }
    }
    else {
        success=false;
    }

    return success;
}


bool testFileCompression(std::string input_path, std::string compressed_path, std::string decompressed_path, int buffer_size, int preset) {

    bool success = true;
    if (!compressLZMA(input_path, compressed_path, buffer_size, preset)) {
        std::cout << "Compression failed!" << std::endl;
        success =false;
    }
    if (!decompressLZMA(compressed_path, decompressed_path, buffer_size, preset)) {
        std::cerr << "Decompression failed!" << std::endl;
        success =false;
    }

    return success;
}

