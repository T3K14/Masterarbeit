#include "simulate.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"

#include <iostream>
#include <map>
// #include <set>
#include <fstream>

using namespace lemon;

int main(int argc, char * argv[]) {

    int number_scenarios = 1;

    bool on_cluster = true;
    bool save_problems = false;

    bool tracking = false;               // ob ich die Algs timen/tracken will (bisher moeglich fuer lp-approx und die beiden optimalen Algs)
    bool save_lp_results = false;


    double c = 3.;

    std::vector<int> nodes = {5,6,7,8,9,10};
    // std::vector<int> nodes = {15, 17};

    // std::vector<int> nodes = {17};
    // int n = 5;

    std::vector<int> runs {20000};

    for (auto n: nodes) {

        NRandomScenarioCreator sc(number_scenarios, rng);    
        RandomTestCreator rtc(0., 10., rng);

        TreePlusC ensemble2(n, sc, rtc, rng, c);
        ensemble2.initialize();

        std::string ordner_name = "RSBC3_" + std::to_string(n) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
        Stoerung st = Stoerung::SingleIncrease;

        rsb_check(runs[0], ensemble2, st, ordner_name, true, false);
    }    
}