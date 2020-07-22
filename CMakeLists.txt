cmake_minimum_required(VERSION 3.1)
project(forageHarvest)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif ()

if (COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
endif (COMMAND cmake_policy)


if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif ()

#set(CMAKE_CXX_STANDARD 11)

#寻找库
find_package(OpenCV REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
#库的包含文件
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

#设置源文件（主文件）
set(SOURCE_FILES main.cpp Detection.cpp Cam.cpp Handle.cpp)
add_executable(forageHarvest ${SOURCE_FILES})
add_definitions(-std=c++14 -O3)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})
else ()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif ()


#设置动态链接库
target_link_libraries(forageHarvest ${OpenCV_LIBS} ${ZED_LIBS})
