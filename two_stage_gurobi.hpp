#pragma once

#include <lemon/list_graph.h>
#include "utilities.hpp"
#include <gurobi_c++.h>
#include "two_stage.hpp"
#include <vector>

void solve_relaxed_lp(TwoStageProblem & two_stage_problem); //, lemon::ListGraph::EdgeMap<std::vector<double>> & result_optimized_values_map);
