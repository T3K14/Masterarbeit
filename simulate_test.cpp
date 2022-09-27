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

    bool seed = true;                   // ob ich vor jeder neuen Probleminstanz einen Set festsetze (zur besseren Reproduzierbarkeit)

    // int number_minus_edges = 20;

    // Alg::GreedyApprox, Alg::Schranke4b, Alg::Optimal, Alg::LPApprox

    NRandomScenarioCreator sc(number_scenarios, rng);    
    // RandomTestCreator rtc(0., 10., rng);
    // HalbNormalCreator hnc(1., rng);
    // GVBilligFirstCreator bfc(0., 10., 0., 1., rng);

    // std::vector<double> ps {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    // std::vector<int> runs {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};//, 50, 50, 50, 50};
    // std::vector<double> ps {0.0};

    // std::vector<int> nodes {5, 10, 20, 40, 80, 160, 320, 640};
    // std::vector<double> cs; //{2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 13., 14., 15., 16., 17., 18., 19., 20., 21., 22., 23., 24., 25.};
    // double cc = 2.0;
    // while (cc < 51.) {
    //     cs.push_back(cc);
    //     cc += 1.0;
    // }
    double c = 4.;

    // double p = 0.5;

    // hier variiere ich die p vom KantenFaktorCreator
    std::vector<double> ps {0., 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.};
    // std::vector<double> ps {0.95};
    // , 0.65, 0.66, 0.67, 0.68, 0.69, 0.7, 0.71, 0.72, 0.73, 0.74, 0.75, 0.76, 0.77, 0.78, 0.79, 0.8
    // std::vector<double> ps {0.54, 0.55, 0.56, 0.57, 0.58, 0.59, 0.6, 0.61, 0.62, 0.63, 0.64, 0.65, 0.66, 0.67, 0.68, 0.69, 0.7, 0.71, 0.72, 0.73, 0.74, 0.75, 0.76, 0.77, 0.78, 0.79, 0.8};
    // std::vector<double> ps;
    // double pp = 0.16;
    // while (pp < 0.4) {
    //     ps.push_back(pp);
    //     pp += 0.01;
    // }
    // pp = 0.81;
    // while (pp < 1.01) {
    //     ps.push_back(pp);
    //     pp += 0.01;
    // }

    // std::vector<double> ps {0.595, 0.605, 0.6125, 0.615, 0.6175, 0.6225, 0.625, 0.6275, 0.6325, 0.635, 0.6375, 0.6425, 0.645, 0.6475, 0.6525, 0.655, 0.6575, 0.6625, 0.665, 0.6675, 0.6725, 0.675, 0.6775, 0.6825, 0.685, 0.6875};
    // std::vector<double> ps {0., 0.025, 0.075, 0.125, 0.175, 0.225, 0.275, 0.325, 0.375, 0.425, 0.475, 0.525, 0.575, 0.625, 0.675, 0.725, 0.775, 0.825, 0.875, 0.925, 0.975, 1.};
    // std::vector<double> ps {0., 0.025, 0.075, 0.125, 0.175, 0.825, 0.875, 0.925, 0.975, 1.};

    // std::vector<double> cs {2.0, 2.01, 2.02, 2.03, 2.04, 2.05, 2.06, 2.07, 2.08, 2.09};

    // std::vector<double> cs; //{2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 13., 14., 15., 16., 17., 18., 19., 20., 21., 22., 23., 24., 25.};
    // double cc = 2.;
    // while (cc < 51.) {
    //     cs.push_back(cc);
    //     cc += 1.0;
    // }

    // while (cc < 21.) {
    //     cs.push_back(cc);
    //     cc += 1.;
    // }

    // std::vector<int> nodes = {320, 640, 1280, 2560};
    std::vector<int> nodes = {8};
    std::vector<int> runs {5000};

    for (auto n: nodes) {
        for (auto p: ps) {

            // std::cout << "p: "<< p << std::endl;

            KantenFaktorCreator2 kfc2(p, 2., rng);
            // RandomTestCreator rtc(0., 10., rng);

            TreePlusC ensemble2(n, sc, kfc2, rng, c);
            // FullyConnected ensemble2(n, sc, rtc);
            ensemble2.initialize();

            // ACHTUNG, LP GUROBI IST NOCH AUF ALT OHNE EC!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            std::set<Alg> alg_set {Alg::Schranke4b, Alg::LPApprox, Alg::Optimal2};
            std::string ordner_name = "LPVLPOPT3_" + std::to_string(n) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
            simulate(runs[0], ensemble2, alg_set, ordner_name, on_cluster, save_problems, tracking, save_lp_results, seed);
        }
    }

    // variiere eine Intervallsbreite fuer beide Intervalle
    // std::vector<double> bs {5.18, 5.2, 5.22, 5.24, 5.26, 5.28, 5.3}; 
    // std::vector<double> bs {0.5, 1.0, 1.5};

    // std::vector<int> nodes = {160, 640, 1280, 2560, 5120};
    // std::vector<int> runs {500};

    // double c = 4.;

    // for (auto n: nodes) {
    //     for (auto b: bs) {

    //     ZweiIntervallGVCreator zic(20.,30., b, b, rng);

    //     TreePlusC ensemble2(n, sc, zic, rng, c);
    //     ensemble2.initialize();

    //     std::set<Alg> alg_set {Alg::GreedyApprox, Alg::Schranke4b};
    //     std::string ordner_name = "ZIC_" + std::to_string(n) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
    //     simulate(runs[0], ensemble2, alg_set, ordner_name, on_cluster, save_problems, tracking, save_lp_results);
    //     }
    // }


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