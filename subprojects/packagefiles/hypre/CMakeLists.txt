# subprojects/packagefiles/hypre/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(hypre-wrapper C CXX)
add_subdirectory(src)

# This file is just used to redirect to the src directory where hypre stores its CMakeLists.txt