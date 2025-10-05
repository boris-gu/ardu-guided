cmake_minimum_required(VERSION 3.10.0)
project(ardu-guided VERSION 0.1.0 LANGUAGES C)

add_executable(ardu-guided main.c)

target_include_directories(ardu-guided PRIVATE ${CMAKE_SOURCE_DIR}/libs)
