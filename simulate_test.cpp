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
    int number_scenarios = 1;

    bool on_cluster = true;
    bool save_problems = false;

    bool tracking = false;               // ob ich die Algs timen/tracken will (bisher moeglich fuer lp-approx und die beiden optimalen Algs)
    bool save_lp_results = true;

    int startseed = 1000000;                   // ob ich vor jeder neuen Probleminstanz einen Set festsetze (zur besseren Reproduzierbarkeit)
    // wenn der seed gleich -1 ist, seede ich nicht, ansonsten wird auf diesen Wert dann in simulate der run-index draufaddiert, damit ich bei wiederholter simulation die selben 
    // Problemstellungen kriege, aber nicht, wenn ich die ids oder N varriiere

    // int number_minus_edges = 20;

    // Alg::GreedyApprox, Alg::Schranke4b, Alg::Optimal, Alg::LPApprox

    NRandomScenarioCreator sc(number_scenarios, rng);    
    // RandomTestCreator rtc(0., 10., rng);
    // HalbNormalCreator hnc(1., rng);
    // GVBilligFirstCreator bfc(0., 10., 0., 1., rng);

    // std::vector<double> ps {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    // std::vector<int> runs {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};//, 50, 50, 50, 50};
    // std::vector<double> ps {0.0};

    // std::vector<double> cs {7.0, 8., 9., 13., 14., 15., 16., 17., 18., 19., 20.}; 

    // std::vector<double> cs {2.25, 2.275, 2.325, 2.35, 2.375, 2.425, 2.45, 2.475, 2.525, 2.55, 2.575, 2.625, 2.65, 2.675, 2.725, 2.75, 2.775, 2.825, 2.85, 2.875}; 


    // std::vector<double> cs;     // {7.0, 8., 9., 13., 14., 15., 16., 17., 18., 19., 20.}; 

    // double cc = 10.;
    // while (cc < 51.) {
    //     cs.push_back(cc);
    //     cc += 1.;
    // }

    // cs.push_back(2.1);
    // cs.push_back(2.5);
    // cs.push_back(3.);
    // cs.push_back(4.);
    // cs.push_back(5.);
    // cs.push_back(6.);

    // cc = 2.1;
    // while (cc < 4.1) {
    //     cs.push_back(cc);
    //     cc += 0.1;
    // }

    // cc = 5.;
    // while (cc < 21.) {
    //     cs.push_back(cc);
    //     cc += 1.;
    // }

    // cs.push_back(20);
    // cs.push_back(40);

    // double c = 4.;

    // double p = 0.5;

    // hier variiere ich die p vom KantenFaktorCreator
    // std::vector<double> ps {0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.};
    // std::vector<double> ps {0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0};
    // , 0.65, 0.66, 0.67, 0.68, 0.69, 0.7, 0.71, 0.72, 0.73, 0.74, 0.75, 0.76, 0.77, 0.78, 0.79, 0.8
    // std::vector<double> ps {0.81, 0.82, 0.83, 0.84, 0.85, 0.86, 0.87, 0.88, 0.89, 0.9, 0.91, 0.92, 0.93, 0.94, 0.95, 0.96, 0.97, 0.98, 0.99, 1.0};
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
    // std::vector<double> ps {0.6725, 0.675, 0.6775, 0.6825, 0.685, 0.6875};
    // std::vector<double> ps {0., 0.025, 0.075, 0.125, 0.175, 0.825, 0.875, 0.925, 0.975, 1.};

    // std::vector<double> cs {2.01, 2.02, 2.03, 2.04, 2.05, 2.06, 2.07, 2.08, 2.09, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0}; //, 5., 6., 7., 8., 9., 10., 11., 12., 13., 14., 15., 16., 17., 18.,19.};

    // std::vector<double> cs {22., 24., 26., 28., 30., 32., 34., 36., 38., 40. };
    // std::vector<double> cs {2.0, 2.01, 2.02, 2.03, 2.04, 2.05, 2.06, 2.07, 2.08, 2.09};
    // std::vector<double> cs {3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0};//, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0};
    std::vector<double> cs {10.};//, 60.,19., 20.}; //, 11., 12., 13., 14., 15., 16., 17., 18.,19.};


    // double cc = 2.;
    // while (cc < 51.) {
    //     cs.push_back(cc);
    //     cc += 1.0;
    // }

    // while (cc < 21.) {
    //     cs.push_back(cc);
    //     cc += 1.;
    // }


    std::vector<int> nodes = {40};
    // std::vector<int> nodes = {10, 20, 40};
    std::vector<int> runs {10000};

    int j = 1;

    for (auto n: nodes) {
        for (auto c: cs) {

            // std::cout << "p: "<< p << std::endl;

            // KantenFaktorCreator2 kfc2(p, 2., rng);
            RandomTestCreator rtc(0., 10., rng);

            TreePlusC ensemble2(n, sc, rtc, rng, c);
            // FullyConnected ensemble2(n, sc, rtc);

            // erstes mal initializen auch seeden!
            rng.seed(j);
            ensemble2.initialize();

            // ACHTUNG, LP GUROBI ALGS SIND MIT EC!

            std::set<Alg> alg_set {Alg::Schranke4b, Alg::GreedyApprox, Alg::LPApprox};
            std::string ordner_name = "CC_" + std::to_string(n) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";

            // ACHTUNG, ICH SEEDE NICHT

            simulate(runs[0], ensemble2, alg_set, ordner_name, on_cluster, save_problems, tracking, save_lp_results, -1);

            j++;
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