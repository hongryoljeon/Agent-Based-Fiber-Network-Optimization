cmake_minimum_required(VERSION 3.10)
project(ElasticNomad_CPP)

# C++ 표준 설정 (filesystem을 위해 17 이상 권장)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 1. 라이브러리 경로 설정 ---
# 캠퍼스 클러스터나 로컬 환경에 따라 라이브러리 경로가 다를 수 있습니다.
# 만약 라이브러리가 표준 경로에 없다면 아래의 주석을 풀고 경로를 지정하세요.
# set(EIGEN3_INCLUDE_DIR "/path/to/eigen")
# set(LBFGSPP_INCLUDE_DIR "/path/to/LBFGSpp/include")

# --- 2. Eigen3 찾기 ---
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
if(Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIRS}")
else()
    # 만약 find_package가 실패할 경우 수동 경로 지정 시도
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

# --- 3. LBFGSpp 포함 ---
# LBFGSpp는 헤더 전용 라이브러리이므로 인클루드 경로만 추가하면 됩니다.
include_directories(${LBFGSPP_INCLUDE_DIR})

# --- 4. 프로젝트 구조 설정 ---
include_directories(include)

# 소스 파일 나열
set(SOURCES
    src/main.cpp
    src/PhysicsEngine.cpp
    src/AgentLogic.cpp
    src/Utils.cpp
)

# 실행 파일 생성
add_executable(ElasticNomad ${SOURCES})

# Eigen3 링크
target_link_libraries(ElasticNomad Eigen3::Eigen)

# --- 5. 성능 최적화 플래그 (중요!) ---
if(MSVC)
    target_compile_options(ElasticNomad PRIVATE /O2 /Oi)
else()
    # GCC/Clang에서 -O3 최적화와 대규모 행렬 연산을 위한 벡터화 플래그
    target_compile_options(ElasticNomad PRIVATE -O3 -march=native -ffast-math)
    # OpenMP를 사용하고 싶다면 아래 주석을 해제하세요.
    # find_package(OpenMP REQUIRED)
    # target_link_libraries(ElasticNomad PUBLIC OpenMP::OpenMP_CXX)
endif()
