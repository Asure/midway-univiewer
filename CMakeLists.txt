cmake_minimum_required(VERSION 3.16)
project(univiewer C)

# SDL2 ---------------------------------------------------------------
# Windows: pass -DSDL2_DIR=<SDL2-devel-VC>/cmake on the command line
#          (build.ps1 does this automatically)
# Linux:   sudo apt install libsdl2-dev
if(WIN32)
    find_package(SDL2 REQUIRED)
else()
    find_package(SDL2 QUIET)
    if(NOT SDL2_FOUND)
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(SDL2 REQUIRED sdl2)
    endif()
endif()

# Target -------------------------------------------------------------
add_executable(univiewer univiewer.c)

target_include_directories(univiewer PRIVATE ${SDL2_INCLUDE_DIRS})
if(TARGET SDL2::SDL2main)
    target_link_libraries(univiewer PRIVATE SDL2::SDL2main SDL2::SDL2)
else()
    target_link_libraries(univiewer PRIVATE ${SDL2_LIBRARIES})
endif()

# Compiler flags -----------------------------------------------------
if(MSVC)
    target_compile_options(univiewer PRIVATE /W3)
    set_target_properties(univiewer PROPERTIES WIN32_EXECUTABLE TRUE)
    target_link_libraries(univiewer PRIVATE comdlg32)
else()
    target_compile_options(univiewer PRIVATE -Wall -Wextra -std=c99)
    target_link_options(univiewer PRIVATE -no-pie)
endif()

# On Windows, copy SDL2.dll next to the exe after build
if(WIN32 AND SDL2_RUNTIME_LIBRARY_DIRS)
    add_custom_command(TARGET univiewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${SDL2_RUNTIME_LIBRARY_DIRS}/SDL2.dll"
            $<TARGET_FILE_DIR:univiewer>)
endif()
