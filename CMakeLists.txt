cmake_minimum_required(VERSION 2.8.3)
project(lidar)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w -pthread")


include_directories(
        /usr/local/include/networktables/
        /usr/local/include/rplidar_sdk/
)

set(SOURCE_FILES
        src/main.cpp
        )

add_executable(lidar ${SOURCE_FILES})
target_link_libraries(lidar
        /usr/local/lib/librplidar_sdk.a
        /usr/local/lib/libntcore.a
        /usr/local/lib/libwpiutil.a
        )