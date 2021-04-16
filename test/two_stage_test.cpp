#include <gtest/gtest.h>
#include <array>
#include <vector>

#include "../two_stage.hpp"
#include "../utilities.hpp"

using namespace lemon;

TEST(UtilitiesSuite, Test1) {

    // test, if the probabilities sum up to almost 1
    auto resVec = calcScenarioProbabilities(8000, rng);

    double sum = 0;
    for (double d : resVec) {
        sum += d;
    }

    ASSERT_NEAR(1.0, sum, 0.0000001);

}

TEST(TwoStageSuite, Test1) {

    // test, if the two stage example MST problem gets the correct solution
    // std::cerr << "ERRREROEROEROEOROEOREOR\n";
    
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

    ListGraph::EdgeMap<int> firstWeights(g); 
    firstWeights[edges[0]] = 7;
    firstWeights[edges[1]] = 5;
    firstWeights[edges[2]] = 9;
    firstWeights[edges[3]] = 8;
    firstWeights[edges[4]] = 7;
    firstWeights[edges[5]] = 5;
    firstWeights[edges[6]] = 15;
    firstWeights[edges[7]] = 6;
    firstWeights[edges[8]] = 8;
    firstWeights[edges[9]] = 9;
    firstWeights[edges[10]] = 11;

    ListGraph::EdgeMap<int> secondWeights(g); 
    secondWeights[edges[0]] = 8;
    secondWeights[edges[1]] = 4;
    secondWeights[edges[2]] = 3;
    secondWeights[edges[3]] = 6;
    secondWeights[edges[4]] = 7;
    secondWeights[edges[5]] = 7;
    secondWeights[edges[6]] = 25;
    secondWeights[edges[7]] = 16;
    secondWeights[edges[8]] = 8;
    secondWeights[edges[9]] = 2;
    secondWeights[edges[10]] = 1;


    auto resSum = twoStageSetting<int>(g, firstWeights, secondWeights, true);

    ASSERT_EQ(21, resSum);


}