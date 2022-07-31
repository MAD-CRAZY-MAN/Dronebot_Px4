cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(camera_control)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(camera_control
    main.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(camera_control
    MAVSDK::mavsdk
    ${OpenCV_LIBS}
)

if(NOT MSVC)
    add_compile_options(camera_control PRIVATE -Wall -Wextra)
else()
    add_compile_options(camera_control PRIVATE -WX -W2)
endif()
