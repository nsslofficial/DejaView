cmake_minimum_required(VERSION 3.5)
project(CompressionProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Find PCL package
find_package(PCL 1.2 REQUIRED)

# Find OpenMP package
find_package(OpenMP REQUIRED)

# Find the LZMA library
find_package(LibLZMA REQUIRED)

find_library(DRACO_LIBRARY NAMES draco)

include_directories(../lib)


# Add source files
set(SOURCES
    ../lib/my_lzma.cpp
    ../lib/my_pcl.cpp
    ../lib/my_utils.cpp
    ../lib/my_draco.cpp
)

# # Add header files
# set(HEADERS
#     my_lzma.h
#     my_pcl.h
# )


# Add executable target
add_executable(compression compression.cpp ${SOURCES})
add_executable(decompression decompression.cpp ${SOURCES})

# Link PCL libraries to your executable
target_link_libraries(compression PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES}  LibLZMA::LibLZMA -Wl,--whole-archive ${DRACO_LIBRARY} -Wl,--no-whole-archive)
target_link_libraries(decompression PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES}  LibLZMA::LibLZMA -Wl,--whole-archive ${DRACO_LIBRARY} -Wl,--no-whole-archive)


