cmake_minimum_required(VERSION 3.16)
project(scfpdp LANGUAGES C CXX)

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
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -fexceptions)
endif()


# -----------------------
# OpenGL
# -----------------------
find_package(OpenGL REQUIRED)

# -----------------------
# Vendor: GLFW
# -----------------------
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(vendor/glfw)

# -----------------------
# Vendor: GLAD
# -----------------------
add_library(glad STATIC
    vendor/glad/src/glad.c
)
target_include_directories(glad PUBLIC
    vendor/glad/include
)

# -----------------------
# Vendor: ImGui
# -----------------------
set(IMGUI_SRC
    vendor/imgui/imgui.cpp
    vendor/imgui/imgui_draw.cpp
    vendor/imgui/imgui_widgets.cpp
    vendor/imgui/imgui_tables.cpp
    vendor/imgui/imgui_demo.cpp
    vendor/imgui/backends/imgui_impl_glfw.cpp
    vendor/imgui/backends/imgui_impl_opengl3.cpp
)

add_library(imgui STATIC ${IMGUI_SRC})

target_include_directories(imgui PUBLIC
    vendor/imgui
    vendor/imgui/backends
)

target_compile_definitions(imgui PUBLIC
    IMGUI_IMPL_OPENGL_LOADER_GLAD
)

target_link_libraries(imgui PUBLIC
    glfw
    glad
    OpenGL::GL
)

# -----------------------
# Source files
# -----------------------
file(GLOB_RECURSE SCFPDP_SOURCES
    src/*.cpp
)

file(GLOB_RECURSE SCFPDP_HEADERS
    src/*.hpp
)

add_executable(scfpdp
    ${SCFPDP_SOURCES}
    ${SCFPDP_HEADERS}    # not strictly needed, but helps IDEs
)

target_include_directories(scfpdp PRIVATE
    src/
)

target_link_libraries(scfpdp PRIVATE imgui)


if(UNIX AND NOT APPLE)
    target_link_libraries(scfpdp PRIVATE pthread)
endif()

install(TARGETS scfpdp RUNTIME DESTINATION bin)
