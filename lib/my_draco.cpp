#include "my_draco.h"


bool convertPCLtoDraco( pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::unique_ptr<draco::PointCloud>& draco_cloud) {

    // Create Draco Point Cloud Builder
    draco::PointCloudBuilder pc_builder;

    int kNumPoints = pcl_cloud->size();
    pc_builder.Start(kNumPoints);
    int pos_att = pc_builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);

    for (draco::PointIndex i(0); i < kNumPoints; ++i) {
        int index= i.value();
        std::vector<float> values= {pcl_cloud->points[index].x, pcl_cloud->points[index].y, pcl_cloud->points[index].z};
        pc_builder.SetAttributeValueForPoint( pos_att, i, values.data());
    }

    // Finalize Draco Point Cloud
    draco_cloud = pc_builder.Finalize(true);
    return true;
}

bool convertDracoToPCL(std::unique_ptr<draco::PointCloud>& draco_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud ) {

    // Retrieve the position attribute from the Draco point cloud
    const draco::PointAttribute *pos_attr = draco_cloud->GetNamedAttribute(draco::GeometryAttribute::POSITION);
    if (!pos_attr) {
        std::cerr << "Position attribute not found in Draco point cloud." << std::endl;
        return false;
    }

    // Prepare storage for position attribute values
    std::vector<float> pos_values(3);

    // Iterate through each point in the Draco point cloud
    for (draco::PointIndex i(0); i < draco_cloud->num_points(); ++i) {
        // Get position attribute values for the current point
        pos_attr->GetMappedValue(i, &pos_values[0]);

        // Create a PCL point and add it to the PCL point cloud
        pcl::PointXYZ pcl_point;
        pcl_point.x = pos_values[0];
        pcl_point.y = pos_values[1];
        pcl_point.z = pos_values[2];
        pcl_cloud->push_back(pcl_point);
    }

    return true;
}

bool initDracoEncoder(draco::Encoder& encoder, int quantization_bits, int compression_level){
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,quantization_bits);
    int speed = 10 - compression_level;
    encoder.SetSpeedOptions(speed, speed);
    return true;
}


bool DracoEncodePointCloudToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, const std::string& output_path,  int quantization_bits, int compression_level, std::vector<uint8_t>& compressedData) {

    draco::Encoder encoder;
    initDracoEncoder(encoder, quantization_bits, compression_level);

    std::unique_ptr<draco::PointCloud> draco_cloud;
    convertPCLtoDraco(pcl_cloud, draco_cloud);

    // Convert to ExpertEncoder that allows us to set per-attribute options.
    std::unique_ptr<draco::ExpertEncoder> expert_encoder;
    expert_encoder.reset(new draco::ExpertEncoder(*draco_cloud));
    expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*draco_cloud));


    draco::EncoderBuffer buffer;

    const draco::Status status = expert_encoder->EncodeToBuffer(&buffer);
    if (!status.ok()) {
        printf("Failed to encode the point cloud.\n");
        printf("%s\n", status.error_msg());
        return false;
    }

    // Save the encoded geometry into a file.
    // if (!draco::WriteBufferToFile(buffer.data(), buffer.size(), output_path)) {
    //     printf("Failed to write the output file.\n");
    //     return false;
    // }

    compressedData.insert(compressedData.end(), buffer.data(), buffer.data()+buffer.size());

    return true;
}


bool DracoDecodePointCloudFromFile(std::vector<char>& data, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud) {

    // std::vector<char> data;
    // if (!draco::ReadFileToBuffer(input_path, &data)) {
    //     printf("Failed opening the input file.\n");
    //     return false;
    // }

    if (data.empty()) {
        printf("Empty input file.\n");
        return false;
    }

    // Create a draco decoding buffer. Note that no data is copied in this step.
    draco::DecoderBuffer buffer;
    buffer.Init(data.data(), data.size());


    // Decode the input data into a geometry.
    std::unique_ptr<draco::PointCloud> draco_cloud;

    auto type_statusor = draco::Decoder::GetEncodedGeometryType(&buffer);
    if (!type_statusor.ok()) {
        return false;
    }

    const draco::EncodedGeometryType geom_type = type_statusor.value();

    if (geom_type == draco::POINT_CLOUD) {
        draco::Decoder decoder;
        auto statusor = decoder.DecodePointCloudFromBuffer(&buffer);
        if (!statusor.ok()) {
            return false;
        }
        draco_cloud = std::move(statusor).value();
    }

    if (draco_cloud == nullptr) {
        printf("Failed to decode the input file.\n");
        return false;
    }

    convertDracoToPCL(draco_cloud, pcl_cloud);

    return true;
}

