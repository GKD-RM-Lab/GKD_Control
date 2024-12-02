cmake_minimum_required(VERSION 3.21)
project(new_infantry)

#set(CMAKE_C_COMPILER clang++)
#set(CMAKE_CXX_COMPILER clang++)

set(CMAKE_CXX_STANDARD 20)

include_directories(./3rdparty/include)
include_directories(./include ./include/chassis)

add_compile_options(-D__DEBUG__)
add_compile_options(-Wno-pointer-arith -Wno-unused-parameter)

file(GLOB_RECURSE SOURCE src/main.cc ./src/*.cc)

add_executable(${PROJECT_NAME} ${SOURCE}
        include/gimbal/gimbal_config.hpp
        src/gimbal/gimbal_temp.cc
        include/gimbal/gimbal_temp.hpp
        include/device/M9025.hpp
        src/device/M9025.cc
        include/gimbal/gimbal_sentry.hpp
        src/gimbal/gimbal_sentry.cpp)

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/lib/libserial.a)
    execute_process(
            COMMAND make
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/src-serial/
    )
    execute_process(
            COMMAND mv ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/src-serial/build/libserial.a ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/lib/
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif ()

target_link_directories(${PROJECT_NAME} PUBLIC ./3rdparty/lib)
target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/lib/libserial.a)
