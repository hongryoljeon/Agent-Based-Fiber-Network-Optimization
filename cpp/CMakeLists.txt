cmake_minimum_required(VERSION 3.10)
# 프로젝트 이름을 레포지토리 이름과 일치시킵니다.
project(Agent-Based-Fiber-Optimization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 라이브러리 경로 설정
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(include)
include_directories(${LBFGSPP_INCLUDE_DIR})

# 소스 파일
set(SOURCES
    src/main.cpp
    src/PhysicsEngine.cpp
    src/AgentLogic.cpp
    src/Utils.cpp
)

# --- 실행 파일 이름을 'FiberSim'으로 변경 ---
add_executable(FiberSim ${SOURCES})

# 링크 설정
target_link_libraries(FiberSim Eigen3::Eigen)

# 최적화 플래그
if(NOT MSVC)
    target_compile_options(FiberSim PRIVATE -O3 -march=native -ffast-math)
endif()
