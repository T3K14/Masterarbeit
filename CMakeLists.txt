cmake_minimum_required(VERSION 3.20.0)

set(first LEMONTEST)
set(This MA)

project(MA C CXX)

set(CMAKE_CXX_STANDARD 17)

enable_testing()
add_subdirectory(googletest)


set(lemonpath D:\\Programme\\Lemon)
# set(lemonreleasepath D:\\Programme\\lemon-1.3.1\\build\\lemon\\Release)
find_library(mylemon lemon PATHS ${lemonpath} NAMES LEMON REQUIRED)

include_directories(D:\\Programme\\Lemon\\include)
# include_directories(${LEMON_INCLUDE_DIRS})
# message("HIHI")

# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
# foreach(dir ${dirs})
#   message(STATUS "dir='${dir}'")
# endforeach()

set (Sources
    two_stage.cpp
    utilities.cpp
)

set (Headers
    two_stage.hpp
    utilities.hpp
)

add_executable(${first} first.cpp)

target_link_libraries(${first} PUBLIC ${mylemon})


add_library(two_stage STATIC ${Sources} ${Headers})
target_link_libraries(two_stage PUBLIC ${mylemon})

add_subdirectory(test)

add_executable(second second.cpp ${Sources} ${Headers})
target_link_libraries(second PUBLIC ${mylemon} two_stage)

