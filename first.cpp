#include <iostream>
#include <lemon/list_graph.h>

int main() {
    lemon::ListDigraph g;
    lemon::ListDigraph::Node u = g.addNode();
    lemon::ListDigraph::Node v = g.addNode();
    lemon::ListDigraph::Arc  a = g.addArc(u, v);
    std::cout << "Hello World! This is LEMON library here." << std::endl;
    std::cout << "We have a directed graph with " << lemon::countNodes(g) << " nodes " << "and " << lemon::countArcs(g) << " arc." << std::endl;
}

