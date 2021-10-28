#include <gtest/gtest.h>
#include <array>
#include <vector>

#include "../two_stage.hpp"
#include "../utilities.hpp"

using namespace lemon;


TEST(TwoStageSuite, Test2) {

    ListGraph g;
    const unsigned nodeNumber = 7;
    std::vector<ListGraph::Node> nodes;
    // std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes.push_back(g.addNode());
    }

    // std::array<ListGraph::Edge, 11> edges;
    std::vector<ListGraph::Edge> edges;

    edges.push_back(g.addEdge(nodes[0], nodes[1]));
    edges.push_back(g.addEdge(nodes[0], nodes[3]));
    edges.push_back(g.addEdge(nodes[1], nodes[3]));
    edges.push_back(g.addEdge(nodes[1], nodes[2]));
    edges.push_back(g.addEdge(nodes[1], nodes[4]));
    edges.push_back(g.addEdge(nodes[2], nodes[4]));
    edges.push_back(g.addEdge(nodes[3], nodes[4]));
    edges.push_back(g.addEdge(nodes[3], nodes[5]));
    edges.push_back(g.addEdge(nodes[4], nodes[5]));
    edges.push_back(g.addEdge(nodes[4], nodes[6]));
    edges.push_back(g.addEdge(nodes[5], nodes[6]));

    std::vector<double> scenarioProbabilities {0.4, 0.6};
    std::vector<double> firstStageWeights {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<std::vector<double>> secondStageWeights {{{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0}, {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0}}};


    UseExternGraphTwoStageMST mst(g, nodes, edges, firstStageWeights, secondStageWeights, scenarioProbabilities);

    auto res = mst.bruteforce();

    // convert to int
    res = res + 0.5;
    int resInt = (int) res;

    ASSERT_EQ(resInt, 6);
}

TEST(TwoStageSuite, Test3) {

    ListGraph g;
    const unsigned nodeNumber = 7;
    std::vector<ListGraph::Node> nodes;
    // std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes.push_back(g.addNode());
    }

    // std::array<ListGraph::Edge, 11> edges;
    std::vector<ListGraph::Edge> edges;

    edges.push_back(g.addEdge(nodes[0], nodes[1]));
    edges.push_back(g.addEdge(nodes[0], nodes[3]));
    edges.push_back(g.addEdge(nodes[1], nodes[3]));
    edges.push_back(g.addEdge(nodes[1], nodes[2]));
    edges.push_back(g.addEdge(nodes[1], nodes[4]));
    edges.push_back(g.addEdge(nodes[2], nodes[4]));
    edges.push_back(g.addEdge(nodes[3], nodes[4]));
    edges.push_back(g.addEdge(nodes[3], nodes[5]));
    edges.push_back(g.addEdge(nodes[4], nodes[5]));
    edges.push_back(g.addEdge(nodes[4], nodes[6]));
    edges.push_back(g.addEdge(nodes[5], nodes[6]));

    std::vector<double> scenarioProbabilities {0.4, 0.6};
    std::vector<double> firstStageWeights {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<std::vector<double>> secondStageWeights {{{.5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5}, {.4, .4, .4, .4, .4, .4, .4, .4, .4, .4, .4}}};

    UseExternGraphTwoStageMST mst(g, nodes, edges, firstStageWeights, secondStageWeights, scenarioProbabilities);
    auto res = mst.bruteforce();
    ASSERT_NEAR(res, 2.64, 0.00000001);
}

TEST(TwoStageSuite, Test4) {

    ListGraph g;
    const unsigned nodeNumber = 7;
    std::vector<ListGraph::Node> nodes;
    // std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes.push_back(g.addNode());
    }

    // std::array<ListGraph::Edge, 11> edges;
    std::vector<ListGraph::Edge> edges;

    edges.push_back(g.addEdge(nodes[0], nodes[1]));
    edges.push_back(g.addEdge(nodes[0], nodes[3]));
    edges.push_back(g.addEdge(nodes[1], nodes[3]));
    edges.push_back(g.addEdge(nodes[1], nodes[2]));
    edges.push_back(g.addEdge(nodes[1], nodes[4]));
    edges.push_back(g.addEdge(nodes[2], nodes[4]));
    edges.push_back(g.addEdge(nodes[3], nodes[4]));
    edges.push_back(g.addEdge(nodes[3], nodes[5]));
    edges.push_back(g.addEdge(nodes[4], nodes[5]));
    edges.push_back(g.addEdge(nodes[4], nodes[6]));
    edges.push_back(g.addEdge(nodes[5], nodes[6]));

    std::vector<double> scenarioProbabilities {0.4, 0.6};
    std::vector<double> firstStageWeights {1.6, .1, 1.2, 4.0, 3.0, .2, .2, .1, 1.0, 1.8, 14.};
    std::vector<std::vector<double>> secondStageWeights {{{.8, 1.5, 3.5, .5, .4, 2.8, 1.5, .8, 17.5, 8.5, 2.7}, {10.4, 8.0, .4, .4, 3.4, 14.4, 2.7, .2, 1.4, 2.4, 3.4}}};

    UseExternGraphTwoStageMST mst(g, nodes, edges, firstStageWeights, secondStageWeights, scenarioProbabilities);
    auto res = mst.bruteforce();
    ASSERT_NEAR(res, 2.8, 0.00000001);
}

// test einmal mit useexternGraphTwoStageMST
TEST(TwoStageSuite, Test5) {

    ListGraph g;
    const unsigned nodeNumber = 4;
    std::vector<ListGraph::Node> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes.push_back(g.addNode());
    }

    std::vector<ListGraph::Edge> edges;

    edges.push_back(g.addEdge(nodes[0], nodes[1]));
    edges.push_back(g.addEdge(nodes[0], nodes[2]));
    edges.push_back(g.addEdge(nodes[0], nodes[3]));
    edges.push_back(g.addEdge(nodes[1], nodes[2]));
    edges.push_back(g.addEdge(nodes[1], nodes[3]));
    edges.push_back(g.addEdge(nodes[2], nodes[3]));

    std::vector<double> scenarioProbabilities {0.4, 0.6};
    std::vector<double> firstStageWeights {1., 1., 10., 10.0, 1.0, 1.};
    std::vector<std::vector<double>> secondStageWeights {{{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}, {1.5, 1.5, 1., 2., 1.5, 1.5}}};

    UseExternGraphTwoStageMST mst(g, nodes, edges, firstStageWeights, secondStageWeights, scenarioProbabilities);
    auto res = mst.bruteforce();
    ASSERT_NEAR(res, 3, 0.00000001);
}

// selber test nochmal mit FullyConnectedTwoStageMST
TEST(TwoStageSuite, Test5b) {

    int numberScenarios = 2;
    unsigned int numberNodes = 4;

    std::vector<double> scenarioProbabilities {0.4, 0.6};
    std::vector<double> firstStageWeights {1., 1., 10., 10.0, 1.0, 1.};
    std::vector<std::vector<double>> secondStageWeights {{{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}, {1.5, 1.5, 1., 2., 1.5, 1.5}}};

    FullyConnectedTwoStageMST mst(numberNodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

    auto res = mst.bruteforce();
    ASSERT_NEAR(res, 3, 0.00000001);
}

TEST(TwoStageSuite, Test6) {

    int numberScenarios = 2;
    unsigned int numberNodes = 4;

    std::vector<double> scenarioProbabilities {0.2, 0.2, 0.6};
    std::vector<double> firstStageWeights {.5, 1., 10., 10.0, 1.0, 1.};
    std::vector<std::vector<double>> secondStageWeights {{{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}, {1.5, 1.5, 1., 2., 1.5, 1.5}, {.5, 1.5, 1.5, .5, 1., .5}}};

    FullyConnectedTwoStageMST mst(numberNodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

    auto res = mst.bruteforce();
    ASSERT_NEAR(res, 2.2, 0.00000001);
    auto res2 = mst.calculate_expected_from_bool_map(mst.bruteforce_first_stage_map);
    std::cout << "die calculate_expected_from_bool_map-methode gibt das zurueck: " << res2 << std::endl;
    ASSERT_NEAR(res2, 2.2, 0.00000001);
}

TEST(TwoStageSuite, Test7) {

    unsigned int numberNodes = 4;

    std::vector<double> scenarioProbabilities {0.397, 0.039, 0.564};
    std::vector<double> firstStageWeights {1.7, .9, 8.4, 4.1, 5.8, 4.1};
    std::vector<std::vector<double>> secondStageWeights {{{3.7, 2.9, 3., 2., .4, 2.1}, {7.3, 5.6, 5.9, 10., 3.1, 7.8}, {9.4, 5.1, 7.6, 6.7, .2, 4.6}}};

    FullyConnectedTwoStageMST mst(numberNodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

    auto res = mst.bruteforce();
    ASSERT_NEAR(res, 2.9925, 0.00000001);

    // teste hier mal noch die calculate_expected_from_bool_map-Funktion
    auto res2 = mst.calculate_expected_from_bool_map(mst.bruteforce_first_stage_map);
    std::cout << "die calculate_expected_from_bool_map-methode gibt das zurueck: " << res2 << std::endl;
    ASSERT_NEAR(res2, 2.9925, 0.00000001);
}

// test nutze edgeWeightIncrease, da muss ich SPAETER NOCH EINEN EIGENEN KONSTRUKTOR FUER SCHREIBEN, DER INTERN edgeWeightIncreas AUFRUFT
/*TEST(TwoStageSuite, Test8) {

    unsigned int numberNodes = 4;

    std::vector<double> scenarioProbabilities {0.397, 0.039, 0.564};
    std::vector<double> firstStageWeights {1., 1., 1., 1., 1., 1.};
    std::vector<std::vector<double>> secondStageWeights {{{3.7, 2.9, 3., 2., .4, 2.1}, {7.3, 5.6, 5.9, 10., 3.1, 7.8}, {9.4, 5.1, 7.6, 6.7, .2, 4.6}}};

    FullyConnectedTwoStageMST mst(numberNodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

    auto res = mst.bruteforce();
    ASSERT_NEAR(res, 2.9925, 0.00000001);
}*/