#include <iostream>
#include <vector>

#include "../two_stage_gurobi.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

using namespace lemon;

int main() {

    // Problemstellung
    ListGraph g;

    const unsigned nodeNumber = 10;
    std::vector<ListGraph::Node> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes.push_back(g.addNode());
    }

    std::vector<ListGraph::Edge> edges;

    edges.push_back(g.addEdge(nodes[0], nodes[1]));//1
    edges.push_back(g.addEdge(nodes[0], nodes[3]));//2

    edges.push_back(g.addEdge(nodes[1], nodes[2]));//3
    edges.push_back(g.addEdge(nodes[1], nodes[3]));//4

    edges.push_back(g.addEdge(nodes[2], nodes[5]));//5
    edges.push_back(g.addEdge(nodes[2], nodes[6]));//6

    edges.push_back(g.addEdge(nodes[3], nodes[4]));//7
    edges.push_back(g.addEdge(nodes[3], nodes[5]));//8

    edges.push_back(g.addEdge(nodes[4], nodes[5]));//9
    edges.push_back(g.addEdge(nodes[4], nodes[7]));//10

    edges.push_back(g.addEdge(nodes[5], nodes[7]));//11
    edges.push_back(g.addEdge(nodes[5], nodes[8]));//12

    edges.push_back(g.addEdge(nodes[6], nodes[8]));//13

    edges.push_back(g.addEdge(nodes[7], nodes[8]));//14

    edges.push_back(g.addEdge(nodes[8], nodes[9]));//15

    std::vector<double> scenarioProbabilities {0.8, 0.2};
    std::vector<double> firstStageWeights                 {20.0, 9.0, 7.0, 15.0, 6.0, 4.0, 14.0, 8.0, 15.0, 18.0, 13.0, 3., 3., 11., 14.};
    std::vector<std::vector<double>> secondStageWeights {{{10.0, 2.0, 7.0, 6.0, 1.0, 10.0, 7.0, 7.0, 18.0, 20.0, 4.0, 3.0, 2.0, 8.0, 10.0}, 
                                                          {22.0, 12.0, 8.0, 16.0, 8.0, 6.0, 25.0, 10.0, 16.0, 2.0, 14.0, 12.0, 6.0, 12.0, 20.0}}};

    // for (int i=0; i<edges.size(); i++) {
    //     secondStageWeights[0][i] += 500;
    //     secondStageWeights[1][i] += 500;
    // }

    UseExternGraphTwoStageMST tsp(g, nodes, edges, firstStageWeights, secondStageWeights, scenarioProbabilities);

    auto res_4b = tsp.do4b();
    auto res_opt = tsp.optimum();

    tsp.greedy();
    // berechne noch den EV, den man mit dieser Loesung erhaelt
    auto res_greedy =  tsp.calculate_expected_from_bool_map(tsp.greedy_first_stage_map);

    std::cout << "Optimale Loesung: " << res_opt << std::endl;

    // auto res_opt2 =  tsp.calculate_expected_from_bool_map(tsp.optimum_first_stage_map);
    // std::cout << "Opt2: " << res_opt2 << std::endl;

    std::cout << "Greedy Loesung: " << res_greedy << std::endl;
    std::cout << "Schranke4b: " << res_4b << std::endl;


    // speichere die output maps
    tsp.save_result_map(tsp.optimum_first_stage_map, "/gss/work/xees8992/Verteidigung/optimum.lgf");
    tsp.save_result_map(tsp.greedy_first_stage_map, "/gss/work/xees8992/Verteidigung/greedy.lgf");

    return 0;
}