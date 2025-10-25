cmake_minimum_required(VERSION 3.16)
project(ddp_solver)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -funroll-loops -ffast-math -flto")
set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(pinocchio REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

# UR10 MuJoCo
add_executable(main_ur10 main_ur10.cpp MujocoUI.cpp)
target_include_directories(main_ur10 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(main_ur10 pinocchio::pinocchio mujoco::mujoco glfw)
