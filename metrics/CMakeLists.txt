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

# Include custom shared library directories, VTK headers, and PCL headers
include_directories(../lib)
include_directories(${VTK_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# Add source files
set(SOURCES
    ../lib/my_pcl.cpp
    ../lib/my_utils.cpp
)

# Add executable targets
add_executable(PSNR PSNR.cpp ${SOURCES})
add_executable(PSNR_fast PSNR_fast.cpp ${SOURCES})
add_executable(chamferDistance chamferDistance.cpp ${SOURCES})
add_executable(chamferDistance_fast chamferDistance_fast.cpp ${SOURCES})
add_executable(RTE RTE.cpp ${SOURCES})
add_executable(RTE_fast RTE_fast.cpp ${SOURCES})

# Link libraries to the executables
target_link_libraries(chamferDistance PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem ${VTK_LIBRARIES} -Wl,--no-whole-archive)
target_link_libraries(chamferDistance_fast PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem ${VTK_LIBRARIES} -Wl,--no-whole-archive)
target_link_libraries(PSNR PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem ${VTK_LIBRARIES} -Wl,--no-whole-archive)
target_link_libraries(PSNR_fast PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem ${VTK_LIBRARIES} -Wl,--no-whole-archive)
target_link_libraries(RTE PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem ${VTK_LIBRARIES} -Wl,--no-whole-archive)
target_link_libraries(RTE_fast PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem ${VTK_LIBRARIES} -Wl,--no-whole-archive)



