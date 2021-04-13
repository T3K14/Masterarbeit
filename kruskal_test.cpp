#include <lemon/kruskal.h>
#include <lemon/list_graph.h>
#include <array>

#include "utilities.hpp"
#include "two_stage.hpp"


using namespace lemon;

int main() {

    ListGraph g;

    const unsigned nodeNumber = 7;
    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    std::array<ListGraph::Edge, 11> edges;

    edges[0] = g.addEdge(nodes[0], nodes[1]);
    edges[1] = g.addEdge(nodes[0], nodes[3]);
    edges[2] = g.addEdge(nodes[1], nodes[3]);
    edges[3] = g.addEdge(nodes[1], nodes[2]);
    edges[4] = g.addEdge(nodes[1], nodes[4]);
    edges[5] = g.addEdge(nodes[2], nodes[4]);
    edges[6] = g.addEdge(nodes[3], nodes[4]);
    edges[7] = g.addEdge(nodes[3], nodes[5]);
    edges[8] = g.addEdge(nodes[4], nodes[5]);
    edges[9] = g.addEdge(nodes[4], nodes[6]);
    edges[10] = g.addEdge(nodes[5], nodes[6]);

    ListGraph::EdgeMap<int> weights(g); 
    weights[edges[0]] = 7;
    weights[edges[1]] = 5;
    weights[edges[2]] = 9;
    weights[edges[3]] = 8;
    weights[edges[4]] = 7;
    weights[edges[5]] = 5;
    weights[edges[6]] = 15;
    weights[edges[7]] = 6;
    weights[edges[8]] = 8;
    weights[edges[9]] = 9;
    weights[edges[10]] = 11;

    ListGraph::EdgeMap<bool> output(g);

    std::cout << "Kruskal_test:\n";
    std::cout << kruskal(g, weights, output) << "\n"; 

    for (int i=0; i<11; i++) {
        std::cout << output[edges[i]] << "\n";
    }
    // std::cout << "\n";








}

