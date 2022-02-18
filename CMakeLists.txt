cmake_minimum_required(VERSION 3.15.0)

set(first LEMONTEST)

project(MA C CXX)

set(CMAKE_CXX_STANDARD 11)

enable_testing()

# Option, damit das mit der Gurobi Bibliothek zusammen laeuft
add_compile_options(-D_GLIBCXX_USE_CXX11_ABI=0)

option(On_Cluster "Switch to on if the code should be build on the HPC-Cluster" OFF)

if(${On_Cluster})
    # muss unter Umstaenden den c und cxx compiler mittels export CXX=/cm/.. und CC=/cm/... aktualisieren

    # nutze die lokale Version, um das math.h Problem zu loesen
    #set(lemonpath "/cm/shared/uniol/software/6.4/lemon/1.2.1-GCCcore-6.4.0")
    set(lemonpath /user/xees8992/local_lemon/lemon-1.3.1/build/lemon)

    include_directories(/user/xees8992/local_lemon/lemon-1.3.1/lemon)   

    # find_library von lemon auf dem Cluster 
    find_library(mylemon PATHS ${lemonpath} NAMES emon.a lemon.a libemon.a)

    # Gurobi, selber gebuilded, damit es keinen Konflikt mit verschiedenen Compilerflags gibt (es gab Error mit _GLIBCXX_USE_CXX11_ABI)
    set(mygurobipath /user/xees8992/Gurobi_src/build/)
    find_library(gurobi_c++ PATHS ${mygurobipath} NAMES gurobi_c++.a gurobi_c++)
    find_library(gurobi91 PATHS ${mygurobipath} NAMES gurobi91)

    message("MESSAGE1: ${gurobi_c++}")
    message("MESSAGE2: ${gurobi91}")
    
    include_directories(/cm/shared/uniol/software/generic/Gurobi/9.1.2/include)
    set(Gurobi_Sources
        two_stage_gurobi.cpp
    )

    set(Gurobi_Headers
        two_stage_gurobi.hpp
    )

    add_library(two_stage_gurobi STATIC ${Gurobi_Sources} ${Gurobi_Headers})
    target_link_libraries(two_stage_gurobi PUBLIC ${mylemon} ${gurobi_c++} ${gurobi91} ${two_stage})
    include(FetchContent)
    FetchContent_Declare(
       googletest
       # Specify the commit you depend on and update it regularly.
       URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
else()
    add_subdirectory(googletest)

    # nicht auf dem Cluster, sondern auf meinem lokalen Windows
    set(lemonpath D:\\Programme\\Lemon)
    include_directories(D:\\Programme\\Lemon\\include)
    find_library(mylemon PATHS ${lemonpath} NAMES lemon emon)

    set(Boost_USE_STATIC_LIBS ON)

    SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "D:\\Program Files (x86)\\boost_1_75_0")
    SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "D:\\Program Files (x86)\\boost_1_75_0\\stage\\lib")
    
    find_package(Boost 1.75.0 COMPONENTS filesystem)


    # set(boostpath D:\\Program Files (x86)\\boost_1_75_0\\stage\\lib)
    # include_directories(D:\\Program Files (x86)\\boost_1_75_0)
    # find_library(myboost PATHS ${boostpath} NAMES boost)


    # message("MESSAGE: ${myboost}")
endif()

set (Sources
    two_stage.cpp
    utilities.cpp
    simulate.cpp
)

set (Headers
    two_stage.hpp
    utilities.hpp
    simulate.hpp
)

option(Compile_Test_Code "Decide if code that is only used for testing functionality ist compiled." ON)

add_executable(${first} first.cpp)
target_link_libraries(${first} PUBLIC ${mylemon})

if(Boost_FOUND)
include_directories(${Boost_INCLUDE_DIRS})

add_library(two_stage STATIC ${Sources} ${Headers})
target_link_libraries(two_stage PUBLIC ${mylemon} ${Boost_LIBRARIES})

endif()
#target_link_libraries(two_stage PUBLIC m)

add_subdirectory(test)
add_subdirectory(simulationen)

add_executable(second second.cpp ${Sources} ${Headers})
target_link_libraries(second PUBLIC ${mylemon} two_stage)

add_executable(4avs4b 4avs4b.cpp ${Source} ${Headers})
target_link_libraries(4avs4b PUBLIC ${mylemon} two_stage)

#add_executable(simulate simulate.cpp ${Sources} ${Headers})
#target_link_libraries(simulate PUBLIC ${mylemon} two_stage)

if(${Compile_Test_Code})

    add_executable(mincut min_cut_test.cpp ${Sources} ${Headers})
    target_link_libraries(mincut PUBLIC ${mylemon})

    add_executable(twiddle twiddle_combi.cpp)

    add_executable(kruskal kruskal_test.cpp ${Sources} ${Headers})
    target_link_libraries(kruskal PUBLIC ${mylemon} two_stage)

    add_executable(update_c_test update_c_test.cpp)
    target_link_libraries(update_c_test PUBLIC two_stage)

endif()

add_executable(simulate2 simulate_test.cpp)
target_link_libraries(simulate2 PUBLIC two_stage)
