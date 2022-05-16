#pragma once

#include <lemon/list_graph.h>
#include "utilities.hpp"
#include <gurobi_c++.h>
#include "two_stage.hpp"
#include <vector>



// nur zum Zeittesten
#include <chrono>
// ich will die zeiten zum optimieren, die Anzahl an Bedingungne, die Setupzeit und die loop-zeit, wo die bedingungn hinzugefuegt werden

double solve_relaxed_lp(TwoStageProblem & two_stage_problem, unsigned long & counter, double & setup_zeit_ms, double & loop_zeit_s, std::vector<double> & opti_times, std::vector<double> & add_constr_times_s);
double solve_relaxed_lp(TwoStageProblem & two_stage_problem);