cmake_minimum_required(VERSION 3.30)
project(copper2)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FILE(GLOB_RECURSE CCM_FILES "src/*.ccm")

add_executable(copper2 main.cpp)
target_sources(copper2 PRIVATE
    FILE_SET CXX_MODULES
        FILES ${CCM_FILES}
)
target_compile_definitions(copper2 PRIVATE FILE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/")
