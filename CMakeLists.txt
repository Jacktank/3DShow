cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)

add_compile_options(-std=c++11)

find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})

find_package(PCL 1.3 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost 1.36.0 REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(frames_show src/frames_show.cpp)
target_link_libraries(frames_show ${PCL_LIBRARIES} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(test_pcd_load src/test_pcd_load.cpp)
target_link_libraries(test_pcd_load ${PCL_LIBRARIES} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(one_frame_show src/one_frame_show.cpp)
target_link_libraries(one_frame_show ${PCL_LIBRARIES})

add_executable(bin2pcd src/bin2pcd.cpp)
target_link_libraries(bin2pcd ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcd2bin src/pcd2bin.cpp)
target_link_libraries(pcd2bin ${PCL_LIBRARIES} ${Boost_LIBRARIES})