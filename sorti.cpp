#include <lemon/list_graph.h>
#include "simulate.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"

int main() {

    lemon::ListGraph g;

    const unsigned nodeNumber = 4;
    std::vector<lemon::ListGraph::Node> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes.push_back(g.addNode());
    }

    std::vector<lemon::ListGraph::Edge> edges;

    edges.push_back(g.addEdge(nodes[0], nodes[1]));
    edges.push_back(g.addEdge(nodes[0], nodes[2]));
    edges.push_back(g.addEdge(nodes[0], nodes[3]));
    edges.push_back(g.addEdge(nodes[1], nodes[2]));
    edges.push_back(g.addEdge(nodes[1], nodes[3]));
    edges.push_back(g.addEdge(nodes[2], nodes[3]));

    std::vector<double> firstStageWeights {1., 16., 4., 3., 2.0, 20.};
    lemon::ListGraph::EdgeMap<double> cost_map(g);

    for (int i=0; i<firstStageWeights.size(); i++) {
        cost_map[edges[i]] = firstStageWeights[i];
    }

    std::cout << "Vor sortieren:\n";
    for (auto e: edges) {
        std::cout << cost_map[e] << "\n";
    }

    std::sort(edges.begin(), edges.end(), [&](lemon::ListGraph::Edge const &a, lemon::ListGraph::Edge const &b) {return cost_map[a] < cost_map[b];});

    std::cout << "\nNach sortieren:\n";
    for (auto e : edges) {
        std::cout << cost_map[e] << "\n";
    }
}