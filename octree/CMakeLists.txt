cmake_minimum_required(VERSION 3.5)
project(CompressionProject)

# Configure C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
# -O3: Enables the highest level of compiler optimizations for maximum execution speed
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Find required dependencies for Point Cloud Library (PCL)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(VTK REQUIRED)

# Include custom shared library directories and VTK headers
include_directories(../lib)
include_directories(${VTK_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# Add source files
set(SOURCES
    ../lib/my_pcl.cpp
    ../lib/my_utils.cpp
)



# Add executable target
add_executable(octree_compression octree_compression.cpp ${SOURCES})
add_executable(octree_compression_fast octree_compression_fast.cpp ${SOURCES})
add_executable(octree_decompression octree_decompression.cpp ${SOURCES})
add_executable(octree_decompression_fast octree_decompression_fast.cpp ${SOURCES})

# Link PCL libraries to your executable
target_link_libraries(octree_compression PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem ${VTK_LIBRARIES})
target_link_libraries(octree_compression_fast PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem ${VTK_LIBRARIES})
target_link_libraries(octree_decompression PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem ${VTK_LIBRARIES})
target_link_libraries(octree_decompression_fast PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem ${VTK_LIBRARIES})


