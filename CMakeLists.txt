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
cmake_minimum_required(VERSION 3.20.1 FATAL_ERROR)

# Project name.
project(essentials
  LANGUAGES CXX
)

# Dependencies directory.
set(PROJECT_DEPS_DIR externals)

# Add executables.
add_library(${PROJECT_NAME} SHARED 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sssp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/externals/mmio.cpp
)

# Include directories.
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    # PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_DEPS_DIR}
)

# Link with the libraries.
# target_link_libraries(${PROJECT_NAME})

# Correct location of the outputs.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

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
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)