cmake_minimum_required(VERSION 3.12)

# Set the project name
project(passivempc VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Option to enable test building or not
option(BUILD_TESTS "Build tests" ON)

# Force coloured compiler output
add_compile_options(-fdiagnostics-color)

# CXX flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Find required packages
if(BUILD_TESTS)
  find_package(Catch2 REQUIRED)
endif()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(NLopt REQUIRED)
find_package(tinyrobotics REQUIRED)
find_package(mpc++ CONFIG REQUIRED)

# Libraries
set(LIBS Eigen3::Eigen ${NLOPT_LIBRARIES} tinyrobotics::tinyrobotics)
include_directories(${mpc++_INCLUDE_DIRS})

# Target names
set(TARGET_LIB passivempc_lib)
set(TARGET_BIN passivempc_executable)
set(TARGET_TEST passivempc_tests)

# Source files
file(GLOB SRC_INCLUDES "include/*.hpp")
file(GLOB SRC "src/*.cpp")
file(GLOB SRC_TEST "test/*.cpp")

# Static library
if(SRC_INCLUDES)
  add_library(${TARGET_LIB} STATIC ${SRC_INCLUDES})
  target_link_libraries(${TARGET_LIB} ${LIBS})
  set_target_properties(${TARGET_LIB} PROPERTIES LINKER_LANGUAGE CXX)
endif()

# src
foreach(SRC_FILE ${SRC})
  get_filename_component(EXAMPLE ${SRC_FILE} NAME_WE)
  add_executable(${EXAMPLE} ${SRC_FILE})
  if(SRC_INCLUDES)
    target_link_libraries(${EXAMPLE} ${TARGET_LIB})
  endif()
  target_link_libraries(${EXAMPLE} ${LIBS})
endforeach()
