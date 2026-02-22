cmake_minimum_required(VERSION 3.10)
project(FiberOptimizer)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_executable(fiber_optimizer
    src/main.cpp
)
