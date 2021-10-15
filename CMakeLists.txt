cmake_minimum_required(VERSION 3.15.0)

set(first LEMONTEST)
set(This MA)

project(MA C CXX)

set(CMAKE_CXX_STANDARD 11)

enable_testing()
add_subdirectory(googletest)


option(On_Cluster "Switch to on if the code should be build on the HPC-Cluster" OFF)

if(${On_Cluster})
    set(lemonpath "/cm/shared/uniol/software/6.4/lemon/1.2.1-GCCcore-6.4.0/lib")
    include_directories("${lemonpath}/include")

    # Gurobi
    set(mygurobipath "/user/xees8992/Gurobi_src/build/libgurobi_c++.a")
    find_library(gurobi_c++ PATHS ${mygurobipath} NAMES gurobi_c++)
    find_library(gurobi91 NAMES gurobi91)

    message("MESSAGE1: ${gurobi_c++}")
    message("MESSAGE2: ${gurobi91}")

    set(Gurobi_Sources
        two_stage_gurobi.cpp
    )

    set(Gurobi_Headers
        two_stage_gurobi.hpp
    )

    add_library(two_stage_gurobi STATIC ${Gurobi_Sources} ${Gurobi_Headers})
    target_link_libraries(two_stage_gurobi PUBLIC ${mylemon})

else()
    set(lemonpath D:\\Programme\\Lemon)
    include_directories(D:\\Programme\\Lemon\\include)
endif()

# option(Compile_Gurobi_Code "Decide if code that makes use of Gurobi is compiled." OFF)


# set(lemonreleasepath D:\\Programme\\lemon-1.3.1\\build\\lemon\\Release)
find_library(mylemon PATHS ${lemonpath} NAMES LEMON REQUIRED)

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

option(Compile_Test_Code "Decide if code that is only used for testing functionality ist compiled." ON)

add_executable(${first} first.cpp)
target_link_libraries(${first} PUBLIC ${mylemon})

add_library(two_stage STATIC ${Sources} ${Headers})
target_link_libraries(two_stage PUBLIC ${mylemon})

add_subdirectory(test)

add_executable(second second.cpp ${Sources} ${Headers})
target_link_libraries(second PUBLIC ${mylemon} two_stage)

add_executable(4avs4b 4avs4b.cpp ${Source} ${Headers})
target_link_libraries(4avs4b PUBLIC ${mylemon} two_stage)


if(${Compile_Test_Code})

    add_executable(mincut min_cut_test.cpp ${Sources} ${Headers})
    target_link_libraries(mincut PUBLIC ${mylemon})

    add_executable(twiddle twiddle_combi.cpp)

    add_executable(kruskal kruskal_test.cpp ${Sources} ${Headers})
    target_link_libraries(kruskal PUBLIC ${mylemon} two_stage)

endif()


