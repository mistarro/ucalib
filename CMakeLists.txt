cmake_minimum_required(VERSION 3.12)

project("ucalib" CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Dependencies

# OpenCV
find_package(OpenCV REQUIRED calib3d core imgcodecs imgproc highgui)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")

# Eigen3
find_package(Eigen3 REQUIRED NO_MODULE)

# Add headers to project
add_custom_target(headers SOURCES
    src/lm.hpp
    src/pattern.hpp
    src/calibration.hpp
    src/debug.hpp
)

#
# Tools
#

# gen-chessboard
add_executable(gen-chessboard
    src/gen-chessboard.cpp
)
target_link_libraries(gen-chessboard opencv_core opencv_highgui)
target_compile_features(gen-chessboard PRIVATE cxx_std_11)

# compile-patterns
add_executable(compile-patterns
    src/compile-patterns.cpp
)
target_link_libraries(compile-patterns opencv_calib3d opencv_core opencv_imgcodecs opencv_imgproc opencv_highgui)
target_compile_features(compile-patterns PRIVATE cxx_std_11)

# calibrate
add_executable(calibrate
    src/calibrate.cpp
    src/calibration.cpp
)
target_link_libraries(calibrate Eigen3::Eigen opencv_core)
target_compile_features(calibrate PRIVATE cxx_std_11)

# intrinsics-test
add_executable(intrinsics-test
    src/intrinsics-test.cpp
    src/calibration.cpp
)
target_link_libraries(intrinsics-test Eigen3::Eigen opencv_core)
target_compile_features(intrinsics-test PRIVATE cxx_std_11)

# correct
add_executable(correct
    src/correct.cpp
    src/calibration.cpp
)
target_link_libraries(correct Eigen3::Eigen opencv_core)
target_compile_features(correct PRIVATE cxx_std_11)

# draw
add_executable(draw
    src/draw.cpp
    src/calibration.cpp
)

target_link_libraries(draw Eigen3::Eigen opencv_core opencv_imgproc opencv_highgui)
target_compile_features(draw PRIVATE cxx_std_11)
