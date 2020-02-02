cmake_minimum_required(VERSION 3.10)
project(motion_tracker)

add_compile_options(-Wall -std=c++17 -pedantic -Wextra -g)
find_package(OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}

        include

        external/cpp-toolkit/include
        external/json/include
        external/crow/include
)


link_libraries(
        ${LIBRARIES}
        ${OpenCV_LIBS}
)

add_library(camera
        src/camera/camera.cpp
        src/camera/camera_frame.cpp
        src/camera/camera_calibration.cpp
        )

target_link_libraries(camera ${Boost_LIBRARIES})

add_executable(app
        src/optic_flow_tracker.cpp
        src/motion_estimation.cpp

        external/cpp-toolkit/src/thread_pool.cpp

        src/web_viewer.cpp

        main.cpp
        )

target_link_libraries(app ${Boost_LIBRARIES} camera dl)

add_executable(calibration
        calibrate_camera.cpp
        )
target_link_libraries(calibration ${Boost_LIBRARIES} camera dl)

