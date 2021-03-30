#include <iostream>
#include <array>
#include <lemon/list_graph.h>
#include "two_stage.hpp"

int main() {

    lemon::ListGraph g;
    
    const unsigned nodeNumber = 10;
    std::array<lemon::ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    RNG rng;
    EdgeCostCreator ecc(rng);

    auto x = g.addEdge(nodes[0], nodes[1]);
    auto y = g.addEdge(nodes[1], nodes[2]);



    auto ptr = ecc.createUniformCosts(g, 1, 99);

    // std::cout << "AXAXAXAXAXAXAXAXAXAXAXAXXAAXAXXAXAXAXAXAXAXXAX\n";


    // for (int i=0; i<nodeNumber; i++) {
    //     std::cout << (*ptr)[nodes[i]] << ", ";

    // }
    // std::cout << "\n";
    

    for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        
        if (e == lemon::INVALID) {
            std::cout << "Invalid\n";
        }
        
        std::cout << (*ptr)[e] << ", ";

    }


    std::cout << "\n";

}