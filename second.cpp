#include <iostream>
#include <array>
#include <lemon/list_graph.h>
#include <lemon/lgf_writer.h>
#include "two_stage.hpp"

int main() {

    lemon::ListGraph g;
    
    const unsigned nodeNumber = 10;
    std::array<lemon::ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    // RNG rng;
    EdgeCostCreator ecc;

    // auto x = g.addEdge(nodes[0], nodes[1]);
    // auto y = g.addEdge(nodes[1], nodes[2]);


    // distribution for selecting a random node 
    std::uniform_int_distribution<int> dist(0, nodeNumber-1);


    const int numberEdges = 50;

    for (int i=0; i<numberEdges; i++) {
        int a = dist(rng);
        int b = dist(rng);
        if (a != b && findEdge(g, g.nodeFromId(a), g.nodeFromId(b)) == lemon::INVALID) {
            g.addEdge(nodes[a], nodes[b]);
        }
    }

    auto ptr = ecc.createUniformCosts(g, 1, 99, rng);

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


    // std::string outputPath("../../output/output.lgf");       // I run second.exe from the 'Code' directory
    lemon::GraphWriter(g, "output.lgf").edgeMap("weights", *ptr).run();


}