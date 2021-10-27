cmake_minimum_required(VERSION 3.15.0)

if(On_Cluster)

add_executable(erste_approx approx.cpp)
target_link_libraries(erste_approx PUBLIC
    two_stage
    two_stage_gurobi
    gurobi_c++	
    gurobi91
)
endif()