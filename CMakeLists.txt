cmake_minimum_required(VERSION 3.0.0)

project(kinect VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_DEBUG_POSTFIX "d")

set(INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Threads REQUIRED)

# take all source files
file(GLOB SRC_FILES *.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/libusb-1.0")
include_directories("/usr/include")
link_directories("/usr/local/x86_64-linux-gnu")

set(DEPENDENCY_LIBS "freenect")

file(MAKE_DIRECTORY ${BUILD_DIR}/bin)

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Core ${DEPENDENCY_LIBS} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${FREENECT_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    DESTINATION ${INSTALL_DIR}
    PUBLIC_HEADER DESTINATION ${EXPORT_DIR})
