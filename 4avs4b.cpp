#include "utilities.hpp"
#include "two_stage.hpp"

#include <array>
#include <iostream>
#include <fstream>

#include <chrono>

using namespace std;
using namespace lemon;

using MapVec = ListGraph::EdgeMap<vector<double>>;


int main() {

    auto t1 = chrono::high_resolution_clock::now();


    // cout << "hi\n";
    const int numberNodes = 6;

    ListGraph g;
    std::array<ListGraph::Node, numberNodes> nodes;

    for(int i=0; i < numberNodes; i++) {
        nodes[i] = g.addNode();
    }

    vector<ListGraph::Edge> edges;

    for (ListGraph::NodeIt n1(g); n1 != INVALID; ++n1) {
        for (ListGraph::NodeIt n2(g); n2 != INVALID; ++n2) {
            if (n1 != n2 && findEdge(g, n1, n2) == INVALID) {
                edges.push_back(g.addEdge(n1, n2));
            }
        }
    }

    ListGraph::EdgeMap<double> firstStageCosts(g);
    for (const auto & edge  : edges) {
        firstStageCosts[edge] = 1.;
    }

    ofstream file;
    file.open(R"(D:\Uni\Masterarbeit\Code\output\text.txt)");
    file << "# p, 4a, 4b\n";


    // std::array<double, 6> ps {0., 0.2, 0.4, 0.6, 0.8, 1.};

    std::array<double, 11> ps {0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.};

    for (double p : ps) {

        cout << "entering loop\n";

        // seed?

        // create scenarios

        MapVec secondStageCosts(g);
        vector<double> secondStageProbabilities;

        edgeWeightIncrease(secondStageCosts, secondStageProbabilities, g, firstStageCosts, p, 2., 1000, rng);
        cout << "scenarios created\n";
        double res4a = bruteForceEnumeration(g, firstStageCosts, secondStageProbabilities, secondStageCosts);
        double res4b = fourb(g, firstStageCosts, secondStageProbabilities, secondStageCosts);
        file << p << " " << res4a << " " << res4b <<"\n";
        cout << "p="<< p << " completed\n";
    }

    auto t2 = chrono::high_resolution_clock::now();
    auto s_int = chrono::duration_cast<chrono::seconds>(t2-t1);
    // chrono::duration<double, chrono::minutes> s_double = t2 - t1;
    cout << "duration: " << s_int.count() << "s\n";


    file.close();
    cout << "finished\n";




    return 0;
}
