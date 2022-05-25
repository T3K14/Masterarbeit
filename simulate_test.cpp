#include "simulate.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"

#include <iostream>
#include <map>
// #include <set>
#include <fstream>

using namespace lemon;

int main(int argc, char * argv[]) {

    // int number_nodes = std::stoi(argv[1]);
    // int number_scenarios = std::stoi(argv[2]);;
    int number_scenarios = 5;

    bool on_cluster = true;
    bool save_problems = false;

    bool tracking = true;               // ob ich die Algs timen/tracken will (bisher moeglich fuer lp-approx und die beiden optimalen Algs)
    bool save_lp_results = true;

    // int number_minus_edges = 20;

    // Alg::GreedyApprox, Alg::Schranke4b, Alg::Optimal, Alg::LPApprox

    NRandomScenarioCreator sc(number_scenarios, rng);    
    // RandomTestCreator rtc(0., 10., rng);
    // HalbNormalCreator hnc(1., rng);
    // GVBilligFirstCreator bfc(0., 10., 0., 1., rng);

    // std::vector<double> ps {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    // std::vector<int> runs {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};//, 50, 50, 50, 50};
    // std::vector<double> ps {0.0};
    std::vector<int> runs {300};

    std::vector<int> nodes = {320};
    // std::vector<int> nodes {5, 10, 20, 40, 80, 160, 320, 640};
    // std::vector<double> cs {2., 2.5, 3., 3.5, 4.};

    // double c = 5.;

    /* DAS NEHME ICH, WENN ICH DIE KNOTEN VARIIERE
    for (auto n: nodes) {
        TreePlusC ensemble2(n, sc, rtc, rng, c);
        ensemble2.initialize();

        std::set<Alg> alg_set {Alg::LPApprox, Alg::LP, Alg::GreedyApprox};
        // std::string ordner_name = "TimeTest_" + std::to_string(number_nodes) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
        std::string ordner_name = "Anteil_C5_" + std::to_string(n) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
        simulate(runs[0], ensemble2, alg_set, ordner_name, on_cluster, save_problems, tracking, save_lp_results);
    }

    */

    // HIER VARIIERE ICH DIE c
    // for (auto c: cs) {
    //     TreePlusC ensemble2(nodes[0], sc, hnc, rng, c);
    //     ensemble2.initialize();

    //     std::set<Alg> alg_set {Alg::LPApprox, Alg::LP, Alg::GreedyApprox, Alg::Schranke4b};
    //     std::string ordner_name = "HalbNormal_" + std::to_string(nodes[0]) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
    //     simulate(runs[0], ensemble2, alg_set, ordner_name, on_cluster, save_problems, tracking, save_lp_results);
    // }

    // hier variiere ich die p vom KantenFaktoCreator
    std::vector<double> ps {0.01, 0.02, 0.05, 0.1};
    double c = 4.;
    for (auto p: ps) {

        KantenFaktorCreator kfc(p, 2., rng);

        TreePlusC ensemble2(nodes[0], sc, kfc, rng, c);
        ensemble2.initialize();

        std::set<Alg> alg_set {Alg::LPApprox, Alg::LP, Alg::GreedyApprox, Alg::Schranke4b};
        std::string ordner_name = "KFC2_extra_" + std::to_string(nodes[0]) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
        simulate(runs[0], ensemble2, alg_set, ordner_name, on_cluster, save_problems, tracking, save_lp_results);
    }

    // for (int i=0; i<ps.size(); i++) {
    //     TreePlusP ensemble2(number_nodes, sc, bfc, rng, ps[i]);
    //     // std::cout << "Edges:" << ensemble2.two_stage_problem.get_number_edges() << std::endl;
    //     // FullyConnected ensemble2(11, sc, rtc);

    //     ensemble2.initialize();
    //     std::set<Alg> s {Alg::Optimal2, Alg::LPApprox};
    //     // std::string ordner_name = "TimeTest_" + std::to_string(number_nodes) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
    //     std::string ordner_name = "TreeTest_" + std::to_string(number_nodes) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
    //     simulate(runs[i], ensemble2, s, ordner_name, on_cluster, save_problems, tracking);
    // }
}