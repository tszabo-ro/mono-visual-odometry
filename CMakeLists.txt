cmake_minimum_required(VERSION 3.16)
project(motion_tracker)

add_compile_options(-Wall -std=c++17 -pedantic -Wextra -g)
find_package(OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}

        include

        external/cpp-toolkit/include
        external/json/single_include
)


link_libraries(
        ${LIBRARIES}
        ${OpenCV_LIBS})


add_executable(app
        src/camera/camera.cpp
        src/camera/camera_frame.cpp

        src/optic_flow_tracker.cpp
        src/motion_estimation.cpp

        external/cpp-toolkit/src/thread_pool.cpp

        main.cpp
        )

target_link_libraries(app ${Boost_LIBRARIES} dl)

add_executable(calibration
        calibrate_camera.cpp
        src/camera/camera.cpp
        src/camera/camera_frame.cpp
        )
target_link_libraries(calibration ${Boost_LIBRARIES} dl)

