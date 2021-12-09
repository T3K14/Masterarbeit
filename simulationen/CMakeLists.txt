cmake_minimum_required(VERSION 3.15.0)

if(On_Cluster)

add_executable(erste_approx approx.cpp)
target_link_libraries(erste_approx PUBLIC
    two_stage
    two_stage_gurobi
    gurobi_c++	
    gurobi91
)

add_executable(approx_long approx_long.cpp)
target_link_libraries(approx_long PUBLIC
    two_stage
    two_stage_gurobi
    gurobi_c++	
    gurobi91
)

endif()

add_executable(erste_bruteforce bruteforce.cpp)
target_link_libraries(erste_bruteforce PUBLIC
two_stage
)