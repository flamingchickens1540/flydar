cmake_minimum_required(VERSION 2.8.3)
project(lidar)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")


include_directories(
        /usr/local/include/ntcore/
        /usr/local/include/rplidar_sdk/
        /usr/local/include/eigen3/
        /usr/include/eigen3/
)

set(SOURCE_FILES
        src/main.cpp
        src/filterinterp.cpp src/common.cpp src/common.h src/filterinterp.h)

add_executable(lidar ${SOURCE_FILES})
target_link_libraries(lidar
        /usr/local/lib/librplidar_sdk.a
        /usr/local/lib/libntcore.a
        /usr/local/lib/libwpiutil.a
        )