cmake_minimum_required(VERSION 3.16)
project(momentum-observer-mujoco)

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

# iiwa14 MuJoCo
add_executable(main_iiwa14 main_iiwa14.cpp MujocoUI.cpp)
target_include_directories(main_iiwa14 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${pinocchio_INCLUDE_DIRS})
target_link_libraries(main_iiwa14 ${pinocchio_LIBRARIES} mujoco::mujoco glfw)
