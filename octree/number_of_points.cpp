#include "my_pcl.h"
#include "my_utils.h"
#include <pcl/compression/octree_pointcloud_compression.h>
#include <chrono>
using namespace std::chrono;


bool number_of_points (std::string dataset_folder)
{

    std::vector<std::filesystem::directory_entry> entries;
    for (const auto& entry : std::filesystem::directory_iterator(dataset_folder)) {
        entries.push_back(entry);
    }

    // for (const auto& entry : std::filesystem::directory_iterator(input_path)) {

    auto start_overall = high_resolution_clock::now();

    float total_points=0.0;
    float total_points_wo_dublicate=0.0;

    #pragma omp parallel for reduction(+:total_points, total_points_wo_dublicate)
    for (size_t i = 0; i < entries.size(); ++i) {
        const auto& entry = entries[i];

        // std::cout << entry.path().filename() << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        // read input cloud
        if(!readPCD(entry.path(), input_cloud))
        {
            std::cout<< "\t"<<"input pcd read fail." << std::endl;
        }
        removeZeroPoints(input_cloud);

        std::vector<pcl::PointXYZ> points(input_cloud->points.begin(), input_cloud->points.end());

        // Sort the points to group duplicates
        std::sort(points.begin(), points.end(), [](const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
            return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
        });

        // Remove duplicates using std::unique
        auto last = std::unique(points.begin(), points.end(), [](const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        });
        points.erase(last, points.end());


        total_points += input_cloud->size();
        total_points_wo_dublicate += points.size();
    }

    total_points = total_points/ (float) entries.size();
    total_points_wo_dublicate = total_points_wo_dublicate/ (float) entries.size();

    auto stop_overall = high_resolution_clock::now();
    auto duration_overall = duration_cast<microseconds>(stop_overall - start_overall);
    // std::cout << "<<<TOTAL_TIME:" << duration_overall.count() << ">>>" << std::endl;

    std::cout << total_points << "\t" << total_points_wo_dublicate << std::endl;


    return true;   
}

int main(int argc, char** argv) {

    std::string dataset_folder      = std::string(argv[1]);


    number_of_points (dataset_folder);

    return 0;
}