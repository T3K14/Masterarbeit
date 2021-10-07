#include <iostream>
#include <lemon/list_graph.h>
#include <lemon/hao_orlin.h>
#include <array>

#include "utilities.hpp"
#include "two_stage.hpp"

using namespace lemon;

int main() {

    ListGraph g;

    const unsigned nodeNumber = 5;
    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    std::array<ListGraph::Edge, 6> edges;

    edges[0] = g.addEdge(nodes[0], nodes[1]);
    edges[1] = g.addEdge(nodes[0], nodes[2]);
    edges[2] = g.addEdge(nodes[1], nodes[2]);
    edges[3] = g.addEdge(nodes[1], nodes[3]);
    edges[4] = g.addEdge(nodes[2], nodes[4]);
    edges[5] = g.addEdge(nodes[3], nodes[4]);

    ListGraph::EdgeMap<double> capacities(g);

    capacities[edges[0]] = 0.8;
    capacities[edges[1]] = 0.1;
    capacities[edges[2]] = 1.1;
    capacities[edges[3]] = 0.05;
    capacities[edges[4]] = 0.7;
    capacities[edges[5]] = 0.1;

    HaoOrlin hao(g, capacities);
    hao.init();
    hao.calculateIn();

    std::cout << "Der min cut Value ist: " << hao.minCutValue() << "\n";
}