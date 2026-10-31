cmake_minimum_required(VERSION 3.20)
project(seismo-node LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(SEISMO_BUILD_TESTS "Build unit tests" ON)
set(SEISMO_SANITIZE "" CACHE STRING "Sanitizers to enable, e.g. address,undefined or thread")

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror)
if(SEISMO_SANITIZE)
  add_compile_options(-fsanitize=${SEISMO_SANITIZE} -fno-omit-frame-pointer)
  add_link_options(-fsanitize=${SEISMO_SANITIZE})
endif()

find_package(Threads REQUIRED)

add_library(seismo STATIC
  src/dsp.cpp
  src/linux_i2c.cpp
  src/lis3dh.cpp
  src/net.cpp
  src/node.cpp
  src/protocol.cpp
  src/sim_sensor.cpp
)
target_include_directories(seismo PUBLIC include)
target_link_libraries(seismo PUBLIC Threads::Threads)

add_executable(seismo-node apps/seismo_node.cpp)
target_link_libraries(seismo-node PRIVATE seismo)

add_executable(seismo-recv apps/seismo_recv.cpp)
target_link_libraries(seismo-recv PRIVATE seismo)

if(SEISMO_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(seismo-tests
    tests/test_dsp.cpp
    tests/test_lis3dh.cpp
    tests/test_node.cpp
    tests/test_protocol.cpp
    tests/test_ring_buffer.cpp
  )
  target_link_libraries(seismo-tests PRIVATE seismo GTest::gtest_main)
  add_test(NAME unit COMMAND seismo-tests)
endif()
