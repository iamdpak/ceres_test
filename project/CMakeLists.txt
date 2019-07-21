cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

project(ceres_check)

find_package(Ceres REQUIRED)

SET(LOCAL_INCLUDE_DIR "include")
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(
${LOCAL_INCLUDE_DIR}
${CERES_INCLUDE_DIRS}
)


include_directories(${LOCAL_INCLUDE_DIR})

add_executable (ceres_check_exe 
src/main.cpp
src/myCeresSolver.cpp )

target_link_libraries (ceres_check_exe ${CERES_LIBRARIES}) 



