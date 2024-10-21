cmake_minimum_required(VERSION 3.20)
project(armorDetect)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(X11 REQUIRED)

include_directories(/opt/MVS/include)
include_directories(include)
add_executable(armorDetect
        myArmorDetect.cpp
        myColorDetectForVideo.cpp
        openCamera.cpp
        beifen.cpp
        src/armor_detect.cpp
        src/CRC_Check.cpp
        src/serialport.cpp
        test.cpp
)

target_link_libraries(armorDetect ${OpenCV_LIBS})
target_link_libraries(armorDetect /opt/MVS/lib/64/libMvCameraControl.so)
