cmake_minimum_required(VERSION 3.0)
project(Outpost)


find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(./include
        ${Boost_INCLUDE_DIRS}
        /usr/local/include/eigen3
        ${Sophus_INCLUDE_DIRS}
        )
set(CMAKE_CXX_STANDARD 17)
add_library(OutpostSim OutpostSim.cpp include/OutpostFilter.h)
add_library(OutpostFilter OutpostFilter.cpp include/OutpostFilter.h)
# add_library(NormalEKF NormalEKF.cpp include/OutpostFilter.h)
add_executable(main main.cpp include/OutpostFilter.h)

target_link_libraries(main
        -lboost_system
        -lboost_filesystem
        -lpthread
        ${Boost_LIBRARIES}
        OutpostSim
        OutpostFilter
       # NormalEKF
        )
