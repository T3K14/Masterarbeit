#include <iostream>
#include <lemon/list_graph.h>
#include <array>
#include <random>

using namespace lemon;

// bool alreadyEdge(const ListGraph & g, int idA, int idB) {

//     for 

//     return false;
// }


int main() {

    const size_t nodeNumber = 30;


    // random generator
    std::random_device seeder;
    const auto seed = seeder.entropy() ? seeder() : time(nullptr);
    std::cout << seed << std::endl;

    std::mt19937 eng(static_cast<std::mt19937::result_type>(seed));
    std::uniform_int_distribution<int> dist(0, nodeNumber-1);

    // std::cout << "HI\n";
    ListGraph g;
    // auto a = g.addNode();
    // auto b = g.addNode();
    // auto e = g.addEdge(a, b);
    // std::cout << countArcs(g) << std::endl;

    // ListGraph::EdgeMap<int> cost(g);
    // cost[e] = 10;
    // std::cout << cost[e] << std::endl;

    // auto a1 = g.direct(e, true);
    // ListGraph::Arc a2 = g.direct(e, false);

    // std::cout << cost[a1] << ", " << cost[a2] << std::endl;

    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    // for (ListGraph::NodeIt n(g); n != INVALID; ++n) {
    //     std::cout << g.id(n) << "\n";
    // }

    // select node pairs at random and add edge if there is no one already
//     for (int i=0; i<100; i++) {
//         std::cout << dist(eng) << "\n";
//     }

    const int numberEdges = 50;

    for (int i=0; i<numberEdges; i++) {
        int a = dist(eng);
        int b = dist(eng);
        if (a != b && findEdge(g, g.nodeFromId(a), g.nodeFromId(b)) == INVALID) {
            g.addEdge(nodes[a], nodes[b]);
        }
    }

    std::cout << countNodes(g) << std::endl;
    std::cout << countEdges(g) << std::endl;

    ListGraph::EdgeMap<int> costMap(g);

    for (ListGraph::EdgeIt e(g); e != INVALID; ++e) {
        costMap[e] = dist(eng);
    }
}