cmake_minimum_required(VERSION 3.16)
project(motion_tracker)

add_compile_options(-Wall -std=c++17 -pedantic -Wextra -g)
find_package(OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
        ${Boost_INCLUDE_DIRS}
        crow/include
        ${OpenCV_INCLUDE_DIRS}
        include
        cpp-toolkit/include
)


link_libraries(
        ${LIBRARIES}
        ${OpenCV_LIBS})


add_executable(app
        src/camera.cpp
        src/camera_frame.cpp

        src/optic_flow_tracker.cpp

        main.cpp
        )
target_link_libraries(app ${Boost_LIBRARIES} dl)
