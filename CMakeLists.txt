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
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -fexceptions -fpermissive)
endif()

# -----------------------
# Detect OpenGL
# -----------------------
find_package(OpenGL QUIET)
if (OpenGL_FOUND)
    set(_SCFPDP_GUI_DEFAULT ON)
    message(STATUS "OpenGL found. GUI frontend enabled by default.")
else()
    set(_SCFPDP_GUI_DEFAULT OFF)
    message(STATUS "OpenGL not found. GUI frontend disabled by default.")
endif()
option(SCFPDP_BUILD_GUI
    "Build GUI frontend (OpenGL/GLFW/ImGui)"
    ${_SCFPDP_GUI_DEFAULT}
)

# -----------------------
# GUI dependencies
# -----------------------
if (SCFPDP_BUILD_GUI)
# OpenGL
    find_package(OpenGL REQUIRED)

# Vendor: GLFW
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    add_subdirectory(vendor/glfw)

# Vendor: GLAD
    add_library(glad STATIC
        vendor/glad/src/glad.c
    )
    target_include_directories(glad PUBLIC
        vendor/glad/include
    )

# Vendor: ImGui
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

add_executable(scfpdp
    ${SCFPDP_SOURCES}
    ${SCFPDP_HEADERS}    # not strictly needed, but helps IDEs
)

target_include_directories(scfpdp PRIVATE
    src/
)

if (SCFPDP_BUILD_GUI)
    target_link_libraries(scfpdp PRIVATE imgui)
    message(STATUS "Building SCFPDP with GUI support.")
else()
    target_compile_definitions(scfpdp PRIVATE SCFPDP_HEADLESS)
    message(STATUS "Building SCFPDP in headless mode (no GUI).")
endif()


if(UNIX AND NOT APPLE)
    target_link_libraries(scfpdp PRIVATE pthread)
endif()

install(TARGETS scfpdp RUNTIME DESTINATION bin)
