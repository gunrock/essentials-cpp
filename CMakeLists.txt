# Installing correct cmake version is easy!
# 1) Find the respective version here;
# https://github.com/Kitware/CMake/releases,
# and 2) replace the [x.xx.x] in the following
# commands with the version number (remove the
# brackets). For example, if you are installing
# CMake 3.22.1, replace [x.xx.x] with 3.22.1:

# wget https://github.com/Kitware/CMake/releases/download/v[x.xx.x]/cmake-[x.xx.x]-linux-x86_64.sh
# chmod +x ./cmake-[x.xx.x]-linux-x86_64.sh
# ./cmake-[x.xx.x]-linux-x86_64.sh
# sudo mv cmake-[x.xx.x]-linux-x86_64 /opt/cmake
# sudo ln -s /opt/cmake/bin/* /usr/local/bin/
cmake_minimum_required(VERSION 3.22.2 FATAL_ERROR)

# Project name.
project(essentials
  LANGUAGES CXX
)

# Dependencies directory.
include(${PROJECT_SOURCE_DIR}/cmake/TBB.cmake)
set(TBB_DIR ${TBB_SOURCE_DIR}/lib/cmake/tbb)
find_package(TBB)
set(PROJECT_DEPS_DIR externals)

# Add executables.
add_library(${PROJECT_NAME} STATIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sssp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/externals/mmio.cpp
)

# Include directories.
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    # PUBLIC ${TBB_INCLUDE_DIR}
    # PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_DEPS_DIR}
)

# Link with the libraries.
target_link_libraries(${PROJECT_NAME}
    PUBLIC TBB::tbb
)

message(${CMAKE_CURRENT_BINARY_DIR})

# Set target properties.
# https://github.com/microsoft/STL/issues/1814
if (MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 19.29.30129 AND CMAKE_VERSION VERSION_GREATER 3.20.3)
    # this change happened in CMake 3.20.4
    set(CMAKE_CXX_STANDARD 23) # /std:c++latest - unlocks the non stable cpp20 features. For new 16.11 versions
    set_target_properties(${PROJECT_NAME}
      PROPERTIES
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin"
    )
else ()
    set(CMAKE_CXX_STANDARD 20) # /std:c++latest for msvc and -std=c++20 for everyone else.
    set_target_properties(${PROJECT_NAME}
      PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin"
    )
endif ()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Install the library.
install(TARGETS ${PROJECT_NAME})

# Add individual executables (programs).
add_executable(sssp ${CMAKE_CURRENT_SOURCE_DIR}/src/sssp.cpp)
target_link_libraries(sssp
  PRIVATE ${PROJECT_NAME}
)

# Correct location of the executables.
set_target_properties(sssp
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin"
)