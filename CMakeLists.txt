cmake_minimum_required(VERSION 3.16)
project(scfpdp LANGUAGES CXX)

# -----------------------
# Global settings
# -----------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable faster builds and warnings
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Recommended warnings for GCC/Clang/MSVC
if (MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# -----------------------
# Source files
# -----------------------
file(GLOB_RECURSE SCFPDP_SOURCES
    src/*.cpp
)

file(GLOB_RECURSE SCFPDP_HEADERS
    src/*.hpp
)

# -----------------------
# Executable
# -----------------------
add_executable(scfpdp
    ${SCFPDP_SOURCES}
    ${SCFPDP_HEADERS}    # not strictly needed, but helps IDEs
)

target_include_directories(scfpdp PRIVATE
    src/
)

# -----------------------
# Optional: link pthread on Linux
# -----------------------
if(UNIX AND NOT APPLE)
    target_link_libraries(scfpdp PRIVATE pthread)
endif()

# -----------------------
# Install target (optional)
# -----------------------
install(TARGETS scfpdp RUNTIME DESTINATION bin)
