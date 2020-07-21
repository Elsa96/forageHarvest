# 设置项目名,项目名即为target名
SET(execName harvest)
PROJECT(${execName})

# 设置最低版本的cmake要求
cmake_minimum_required(VERSION 3.1)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

# 主版本号
set(VERSION_MAJOR 2)
# 副版本号
set(VERSION_MINOR 1)
# 补丁号
set(VERSION_PATCH 0)


# 指定使用的编译器
set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# 设置 C语言标准
set(CMAKE_C_STANDARD 99)  # 90, 99, 11
# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)  # 98, 11, 14, 17, 20.

# 添加编译参数
#add_definitions(-std=c++11 -Wall -g -O2)
#add_definitions( “-Wall -g -O2 -ansi –pedantic”)

# 设置编译选项
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -Wall -g -O2")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -O2")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# 设置编译模式,debug会输出调试信息,Debug版会使用参数-g；Release版使用-O3 –DNDEBUG
set(CMAKE_BUILD_TYPE "Debug")
#set( CMAKE_BUILD_TYPE "Release" )

# 添加头文件路径
include_directories(./ ./include ./src ./qt ./thirdparty ./thirdparty/percipio ./thirdparty/zed)

# 添加源文件路径 -> 变量DIR_SRCS
# 查找当前目录中的所有源文件，并将结果列表存放在变量DIR_SRCS中
aux_source_directory(./src SRC_DIR_SRC)
# 可以添加多个源文件目录
aux_source_directory(./ SRC_DIR_CURT)
aux_source_directory(./thirdparty/zed SRC_DIR_Third)
set(SRC_FILES ${SRC_DIR_SRC} ${SRC_DIR_CURT} ${SRC_DIR_Third})

# 匹配搜索文件
#FILE(GLOB srcs src/*.cpp src/*.h*)
#FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
#FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

# 添加CMakeList子目录,# 告诉cmake去子目录寻找新的CMakeList.txt子文件 # 子目录 CMakeLists.txt 将从父目录 CMakeLists.txt 继承设置
#add_subdirectory()


# --- ZED ---
find_package(ZED 2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
# support GLViewer
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

SET(SPECIAL_OS_LIBS "")  #"pthread" "X11"
SET(SPECIAL_OS_LIBS "pthread" "X11")
add_definitions(-Wno-format-extra-args)
add_definitions(-Wno-write-strings -fpermissive)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS}) # support GLViewer
include_directories(${GLUT_INCLUDE_PATH})
include_directories(${OpenGL_INCLUDE_PATH})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${GLEW_LIBRARY_DIRS}) # support GLViewer
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

if(LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()
set(ZED_LIB ${ZED_LIBS} ${SPECIAL_OS_LIBS} ${OpenCV_LIBRARIES}
        ${GLEW_LIBRARY} # support GLViewer
        ${GLUT_LIBRARY}
        ${OPENGL_LIBRARIES})


# --- Qt ---
#设置 cmake
# Find includes in corresponding build directories
# 自动添加CMAKE_CURRENT_BINARY_DIR和CMAKE_CURRENT_SOURCE_DIR
#设置工程包含当前目录,使找到编译的ui_*.h
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 打开Qt的MOC功能和UIC文件处理功能
# To automatically run MOC when building (Meta Object Compiler)
#打开全局moc,设置自动生成moc文件,moc必须
set(CMAKE_AUTOMOC ON)
# To automatically run UIC when building (User Interface Compiler)
#打开全局uic，uic必须
set(CMAKE_AUTOUIC ON)
# To automatically run RCC when building (Resource Compiler)
#打开全局rcc，qrc非必须
set(CMAKE_AUTORCC ON)
#set(CMAKE_PREFIX_PATH "/usr/bin/cmake")
#set(CMAKE_PREFIX_PATH "/home/elsa/Qt/Qt5.12.0/5.12.0/gcc_64/lib/cmake")
#查找需要的Qt库文件
find_package(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5Gui)
#find_package(Qt5OpenGL)
#find_package(Qt5Xml)
# Take care of Qt dependencies
#find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)


#查找当前文件夹/qt中的所有源代码文件，也可以通过Set命令将所有文件设置为一个变量
FILE(GLOB Qt_CPP_FILES "./qt/*.cpp")
#查找设置当前文件夹/qt中所有的头文件
FILE(GLOB Qt_HEAD_FILES "./qt/*.h*")  # 为啥找不到.h
#查找设置当前文件夹/qt中所有的ui文件
FILE(GLOB ui_FILES "./qt/*.ui")
#查找设置当前文件夹/qt中所有的.qrc文件
FILE(GLOB qrc_FILES "./qt/*.qrc")

# 应该不需要??
#QT5_WRAP_CPP(HEADERS_MOC ${INCLUDE_FILES})
#添加 窗体.ui文件,把.ui文件包裹成一个ui_*.h文件，必须
qt5_wrap_ui(Qt_UI_FILES ${ui_FILES})
#qt5_wrap_ui(Qt_UI_FILES mainwindow.ui)
#添加 资源文件.qrc，生成qrc_*.cpp,非必须
qt5_add_resources(Qt_QRC_FILES ${qrc_FILES})
#qt5_add_resources(qrc_FILES drawable.qrc)

# 上面不知为啥找不到mainwindow.h
#include_directories(./qt)

# Qt 源文件
set(QT_SRC_FILES ${Qt_CPP_FILES} ${Qt_HEAD_FILES} ${Qt_UI_FILES} ${Qt_QRC_FILES})

# 设置Qt链接库别名
set(QT_LIB Qt5::Widgets Qt5::Core Qt5::Gui)

#将ui文件和生成文件整理在一个文件夹中，非必须
#source_group("Ui" FILES ${ui_FILES} ${Qt_UI_FILES})


# --- OpenCV ---
# 依赖 (如未编译装opencv)
# sudo apt-get install libopencv-dev  # 3.2
# Find an external project, and load its settings.
find_package(OpenCV REQUIRED)  # 寻找OpenCV库
#find_package(OpenCV 3.3.1 REQUIRED)  # 指定版本
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV library not found")
else()
    include_directories(${OpenCV_INCLUDE_DIRS})  # opencv添加头文件
endif()
# 链接OpenCV库
#target_link_libraries(xxx ${OpenCV_LIBS})

# --- PCL ---
find_package(PCL REQUIRED)
if(NOT PCL_FOUND)
    message(WARNING "PCL library not found, some samples will not be compiled")
else()
    include_directories(${PCL_INCLUDE_DIRS})
    #向工程添加多个特定的头文件搜索路径,路径之间用空格分隔
    add_definitions(${PCL_DEFINITIONS})
    #add_definitions 为源文件的编译添加由-D引入的define flag
endif()


# 设置环境变量
#set(Camera_LIB tycam) # TY camera
set(Thread_LIB pthread)


# 指定可执行文件输出目录
#SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
#SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)  # 可执行软件的目录
#SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)  # 存放动态库的文件夹位置
#SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)  # 存放静态库的文件夹位置
# 指定库文件输出目录
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)  # 默认存放库文件的位置


# 主函数生成可执行文件
add_executable(${PROJECT_NAME} ${SRC_FILES} ${QT_SRC_FILES})


# 增加库的搜索路径,动态链接库或静态链接库的搜索路径,this command is rarely necessary
#link_directories(./lib)
# All targets link with the same set of libs ??
#link_libraries(lib1 lib2 ...)

# 递归获取目录下所有的C文件
#file(GLOB_RECURSE c_files ./*.c)
# 递归获取目录下所有的h文件
#file(GLOB_RECURSE h_files ./*.h)
# 生成动态库和静态库
#add_library(add_lib_shared SHARED ${c_files})
#add_library(add_lib_static STATIC ${c_files})
# 将动态库和静态库的名字设置为 myadd
#set_target_properties(add_lib_shared PROPERTIES OUTPUT_NAME "myadd")
#set_target_properties(add_lib_static PROPERTIES OUTPUT_NAME "myadd")
# 设置动态库版本
#set_target_properties(add_lib_shared PROPERTIES VERSION 1.0 SOVERSION 1)
# 安装动态库和静态库
#INSTALL(TARGETS add_lib_shared add_lib_static
#        LIBRARY DESTINATION lib
#        ARCHIVE DESTINATION lib)
#安装头文件
#INSTALL(FILES ${h_files} DESTINATION include)

# 生成自己的链接库
# 可以是动态(SHARED)或者静态库(STATIC)
#add_library(helloLIB SHARED hello.cpp)  # 将hello.cxx编译成动态库如libHelloLIB.so
#add_library(helloLIB STATIC hello.cpp)  # 将hello.cxx编译成静态库如libHelloLIB.a

# 指定生成对象时依赖的库,设置链接库,为target添加需要链接的共享库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${QT_LIB} ${Thread_LIB} ${ZED_LIB} -lGL -lglut -lGLEW) # support GLViewer -lGL -lglut -lGLEW
# lots of properties... OUTPUT_NAME, VERSION, ....
#set_target_properties(ss7gw PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}")

# 默认支持 make clean

# 安装
# 设置安装的路径
set(CMAKE_INSTALL_PREFIX ~/harvest)
# TARGETS安装目录,DESTINATION为相对于CMAKE_INSTALL_PREFIX的值的相对路径
INSTALL(TARGETS ${Target_Name}
        RUNTIME DESTINATION bin  # 可执行二进制
        LIBRARY DESTINATION lib  # 动态库
        ARCHIVE DESTINATION libstatic # 静态库
        )
# FILES将给定的文件复制到指定目录
INSTALL(FILES README.md DESTINATION ./)
#INSTALL(FILES Rt.txt height.txt parameter.json param.txt high.txt DESTINATION ./)
# DIRECTORY复制文件夹
install(DIRECTORY "${PROJECT_SOURCE_DIR}/bin" DESTINATION ./
        FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/res" DESTINATION ./)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/qt" DESTINATION ./)
#install(DIRECTORY "${PROJECT_SOURCE_DIR}/image" DESTINATION ./)
#FILES_MATCHING PATTERN "*.gif"  # 文件过滤
#               PATTERN "CVS" EXCLUDE ) # 排除路径

#INSTALL(FILES image/* DESTINATION image)
# PROGRAMS安装外部或者已编译好的库或者可执行文件,shell脚本
#install(PROGRAMS src/lib/boost/libboost_system.so DESTINATION lib)
# 安装时执行CODE代码
install(CODE "MESSAGE(\"Install done .\")")
# 生成并安装一个包含将安装过程的安装树导入到另一个工程中的CMake文件
#install(EXPORT myproj NAMESPACE mp_ DESTINATION lib/myproj)

# 生成安装包??
# 构建一个 CPack 安装包
# 导入 InstallRequiredSystemLibraries 模块
include(InstallRequiredSystemLibraries)
#设置一些 CPack 相关变量，包括版权信息和版本信息

#SET(CPACK_CMAKE_GENERATOR "Unix Makefiles")

#设置协议信息
#set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/License.txt")
# 设置软件包名称
SET(CPACK_PACKAGE_NAME "CMakeDemo")

#SET(CPACK_PACKAGE_VENDOR "Kitware")
# 设置软件包版本
SET(CPACK_PACKAGE_VERSION "2.5.0")
SET(CPACK_PACKAGE_VERSION_MAJOR "2")
SET(CPACK_PACKAGE_VERSION_MINOR "5")
SET(CPACK_PACKAGE_VERSION_PATCH "0")

SET(CPACK_SYSTEM_NAME "Linux-i686")
SET(CPACK_TOPLEVEL_TAG "Linux-i686")

#设置文件打包文件名称
set(PROJECT_VERSION_FULL ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})
set(CPACK_SOURCE_PACKAGE_FILE_NAME sweather-${PROJECT_VERSION_FULL})

#设置软件包装位置
set(CPACK_SET_DESTDIR true)
set(CPACK_INSTALL_PREFIX "/opt/omatrix/")
#设置打包文件生成位置
set(CPACK_PACKAGE_DIRECTORY ${PROJECT_SOURCE_DIR}/pack)
#设置软件包生成的位置
set(CPACK_OUTPUT_FILE_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
#设置忽略文件
set(CPACK_SOURCE_IGNORE_FILES
        ${PROJECT_SOURCE_DIR}/build
        ${PROJECT_SOURCE_DIR}/cmake-build-debug
        ${PROJECT_SOURCE_DIR}/pack
        ${PROJECT_SOURCE_DIR}/.idea
        ${PROJECT_SOURCE_DIR}/.git
        ${PROJECT_SOURCE_DIR}/.gitignore
        ${PROJECT_SOURCE_DIR}/.vscode
        )
# 设置包类型
SET(CPACK_GENERATOR "TGZ")  # TGZ(.tar.gz)

# 导入 CPack 模块
include(CPack)

#打印信息
MESSAGE(STATUS "Project Name : ${PROJECT_NAME}")
MESSAGE(STATUS "Project Dirrectory : ${PROJECT_SOURCE_DIR}")
MESSAGE(STATUS "Default Install Dirrectory : ${CMAKE_INSTALL_PREFIX}")
