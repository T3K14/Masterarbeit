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

TEST(TwoStageSuite, Test2) {

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


    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = 1.;
    firstWeights[edges[1]] = 1.;
    firstWeights[edges[2]] = 1.;
    firstWeights[edges[3]] = 1.;
    firstWeights[edges[4]] = 1.;
    firstWeights[edges[5]] = 1.;
    firstWeights[edges[6]] = 1.;
    firstWeights[edges[7]] = 1.;
    firstWeights[edges[8]] = 1.;
    firstWeights[edges[9]] = 1.;
    firstWeights[edges[10]] = 1.;


    // ListGraph::EdgeMap<double> secondWeights1(g); 
    // secondWeights1[edges[0]] = 2.;
    // secondWeights1[edges[1]] = 2.;
    // secondWeights1[edges[2]] = 2.;
    // secondWeights1[edges[3]] = 2.;
    // secondWeights1[edges[4]] = 2.;
    // secondWeights1[edges[5]] = 2.;
    // secondWeights1[edges[6]] = 2.;
    // secondWeights1[edges[7]] = 2.;
    // secondWeights1[edges[8]] = 2.;
    // secondWeights1[edges[9]] = 2.;
    // secondWeights1[edges[10]] = 2.;


    // ListGraph::EdgeMap<double> secondWeights2(g); 
    // secondWeights2[edges[0]] = 3.;
    // secondWeights2[edges[1]] = 3.;
    // secondWeights2[edges[2]] = 3.;
    // secondWeights2[edges[3]] = 3.;
    // secondWeights2[edges[4]] = 3.;
    // secondWeights2[edges[5]] = 3.;
    // secondWeights2[edges[6]] = 3.;
    // secondWeights2[edges[7]] = 3.;
    // secondWeights2[edges[8]] = 3.;
    // secondWeights2[edges[9]] = 3.;
    // secondWeights2[edges[10]] = 3.;

    std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> scenarioSecondStageCosts;
    // scenarioSecondStageCosts.push_back(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&secondWeights1));
    
    // std::make_unique<lemon::ListGraph::EdgeMap<double>>
    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));
    // (*scenarioSecondStageCosts[0]).reset(&secondWeights1);

    (*scenarioSecondStageCosts[0])[edges[0]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[1]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[2]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[3]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[4]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[5]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[6]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[7]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[8]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[9]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[10]] = 2.;

    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

    (*scenarioSecondStageCosts[1])[edges[0]] = 3.;
    (*scenarioSecondStageCosts[1])[edges[1]] = 3.;
    (*scenarioSecondStageCosts[1])[edges[2]] = 3.;
    (*scenarioSecondStageCosts[1])[edges[3]] = 3.;
    (*scenarioSecondStageCosts[1])[edges[4]] = 3.;
    (*scenarioSecondStageCosts[1])[edges[5]] = 3.;
    (*scenarioSecondStageCosts[1])[edges[6]] = 3.;
    (*scenarioSecondStageCosts[1])[edges[7]] = 3.;
    (*scenarioSecondStageCosts[1])[edges[8]] = 3.;
    (*scenarioSecondStageCosts[1])[edges[9]] = 3.;
    (*scenarioSecondStageCosts[1])[edges[10]] = 3.;

    // scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));
    // scenarioSecondStageCosts[1].reset(&secondWeights2);
    // scenarioSecondStageCosts.push_back(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&secondWeights2));

    // std::cerr << (* ((*scenarioSecondStageCosts[0])))[edges[0]] << std::endl;

    // // scenarioSecondStageCosts.push_back(secondWeights1);
    std::vector<double> probs{0.4, 0.6};
    auto res = bruteForceEnumeration(g, firstWeights, probs, scenarioSecondStageCosts);

    // convert to int
    res = res + 0.5;
    int resInt = (int) res;

    ASSERT_EQ(resInt, 6);

    // ASSERT_EQ(1,2);
}

TEST(TwoStageSuite, Test3) {

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


    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = 1.;
    firstWeights[edges[1]] = 1.;
    firstWeights[edges[2]] = 1.;
    firstWeights[edges[3]] = 1.;
    firstWeights[edges[4]] = 1.;
    firstWeights[edges[5]] = 1.;
    firstWeights[edges[6]] = 1.;
    firstWeights[edges[7]] = 1.;
    firstWeights[edges[8]] = 1.;
    firstWeights[edges[9]] = 1.;
    firstWeights[edges[10]] = 1.;


    // ListGraph::EdgeMap<double> secondWeights1(g); 
    // secondWeights1[edges[0]] = 2.;
    // secondWeights1[edges[1]] = 2.;
    // secondWeights1[edges[2]] = 2.;
    // secondWeights1[edges[3]] = 2.;
    // secondWeights1[edges[4]] = 2.;
    // secondWeights1[edges[5]] = 2.;
    // secondWeights1[edges[6]] = 2.;
    // secondWeights1[edges[7]] = 2.;
    // secondWeights1[edges[8]] = 2.;
    // secondWeights1[edges[9]] = 2.;
    // secondWeights1[edges[10]] = 2.;


    // ListGraph::EdgeMap<double> secondWeights2(g); 
    // secondWeights2[edges[0]] = 3.;
    // secondWeights2[edges[1]] = 3.;
    // secondWeights2[edges[2]] = 3.;
    // secondWeights2[edges[3]] = 3.;
    // secondWeights2[edges[4]] = 3.;
    // secondWeights2[edges[5]] = 3.;
    // secondWeights2[edges[6]] = 3.;
    // secondWeights2[edges[7]] = 3.;
    // secondWeights2[edges[8]] = 3.;
    // secondWeights2[edges[9]] = 3.;
    // secondWeights2[edges[10]] = 3.;

    std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> scenarioSecondStageCosts;
    // scenarioSecondStageCosts.push_back(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&secondWeights1));
    
    // std::make_unique<lemon::ListGraph::EdgeMap<double>>
    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));
    // (*scenarioSecondStageCosts[0]).reset(&secondWeights1);

    (*scenarioSecondStageCosts[0])[edges[0]] = 0.5;
    (*scenarioSecondStageCosts[0])[edges[1]] = 0.5;
    (*scenarioSecondStageCosts[0])[edges[2]] = 0.5;
    (*scenarioSecondStageCosts[0])[edges[3]] = .5;
    (*scenarioSecondStageCosts[0])[edges[4]] = .5;
    (*scenarioSecondStageCosts[0])[edges[5]] = .5;
    (*scenarioSecondStageCosts[0])[edges[6]] = .5;
    (*scenarioSecondStageCosts[0])[edges[7]] = .5;
    (*scenarioSecondStageCosts[0])[edges[8]] = .5;
    (*scenarioSecondStageCosts[0])[edges[9]] = .5;
    (*scenarioSecondStageCosts[0])[edges[10]] = .5;

    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

    (*scenarioSecondStageCosts[1])[edges[0]] = .4;
    (*scenarioSecondStageCosts[1])[edges[1]] = .4;
    (*scenarioSecondStageCosts[1])[edges[2]] = .4;
    (*scenarioSecondStageCosts[1])[edges[3]] = .4;
    (*scenarioSecondStageCosts[1])[edges[4]] = .4;
    (*scenarioSecondStageCosts[1])[edges[5]] = .4;
    (*scenarioSecondStageCosts[1])[edges[6]] = .4;
    (*scenarioSecondStageCosts[1])[edges[7]] = .4;
    (*scenarioSecondStageCosts[1])[edges[8]] = .4;
    (*scenarioSecondStageCosts[1])[edges[9]] = .4;
    (*scenarioSecondStageCosts[1])[edges[10]] = .4;

    // scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));
    // scenarioSecondStageCosts[1].reset(&secondWeights2);
    // scenarioSecondStageCosts.push_back(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&secondWeights2));

    // std::cerr << (* ((*scenarioSecondStageCosts[0])))[edges[0]] << std::endl;

    // // scenarioSecondStageCosts.push_back(secondWeights1);
    std::vector<double> probs{0.4, 0.6};
    auto res = bruteForceEnumeration(g, firstWeights, probs, scenarioSecondStageCosts);

    // convert to int
    // res = res + 0.5;
    // int resInt = (int) res;

    // ASSERT_EQ(resInt, 6);

    // ASSERT_EQ(1,2);
    ASSERT_NEAR(res, 2.64, 0.00000001);
}

TEST(TwoStageSuite, Test4) {

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


    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = 1.6;
    firstWeights[edges[1]] = .1;
    firstWeights[edges[2]] = 1.2;
    firstWeights[edges[3]] = 4.;
    firstWeights[edges[4]] = 3.;
    firstWeights[edges[5]] = 0.2;
    firstWeights[edges[6]] = 0.2;
    firstWeights[edges[7]] = 0.1;
    firstWeights[edges[8]] = 1.;
    firstWeights[edges[9]] = 1.8;
    firstWeights[edges[10]] = 14.;

    std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> scenarioSecondStageCosts;
    // scenarioSecondStageCosts.push_back(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&secondWeights1));
    
    // std::make_unique<lemon::ListGraph::EdgeMap<double>>
    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));
    // (*scenarioSecondStageCosts[0]).reset(&secondWeights1);

    (*scenarioSecondStageCosts[0])[edges[0]] = 0.8;
    (*scenarioSecondStageCosts[0])[edges[1]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[2]] = 3.5;
    (*scenarioSecondStageCosts[0])[edges[3]] = .5;
    (*scenarioSecondStageCosts[0])[edges[4]] = 0.4;
    (*scenarioSecondStageCosts[0])[edges[5]] = 2.8;
    (*scenarioSecondStageCosts[0])[edges[6]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[7]] = 0.8;
    (*scenarioSecondStageCosts[0])[edges[8]] = 17.5;
    (*scenarioSecondStageCosts[0])[edges[9]] = 8.5;
    (*scenarioSecondStageCosts[0])[edges[10]] = 2.7;

    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

    (*scenarioSecondStageCosts[1])[edges[0]] = 10.4;
    (*scenarioSecondStageCosts[1])[edges[1]] = 8.0;
    (*scenarioSecondStageCosts[1])[edges[2]] = .4;
    (*scenarioSecondStageCosts[1])[edges[3]] = .4;
    (*scenarioSecondStageCosts[1])[edges[4]] = 3.4;
    (*scenarioSecondStageCosts[1])[edges[5]] = 14.4;
    (*scenarioSecondStageCosts[1])[edges[6]] = 2.7;
    (*scenarioSecondStageCosts[1])[edges[7]] = 0.2;
    (*scenarioSecondStageCosts[1])[edges[8]] = 1.4;
    (*scenarioSecondStageCosts[1])[edges[9]] = 2.4;
    (*scenarioSecondStageCosts[1])[edges[10]] = 3.4;

    // scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));
    // scenarioSecondStageCosts[1].reset(&secondWeights2);
    // scenarioSecondStageCosts.push_back(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&secondWeights2));

    // std::cerr << (* ((*scenarioSecondStageCosts[0])))[edges[0]] << std::endl;

    // // scenarioSecondStageCosts.push_back(secondWeights1);
    std::vector<double> probs{0.4, 0.6};
    auto res = bruteForceEnumeration(g, firstWeights, probs, scenarioSecondStageCosts);

    // convert to int
    // res = res + 0.5;
    // int resInt = (int) res;

    // ASSERT_EQ(resInt, 6);

    // ASSERT_EQ(1,2);
    ASSERT_NEAR(res, 2.8, 0.00000001);
}

TEST(TwoStageSuite, Test5) {

    ListGraph g;
    const unsigned nodeNumber = 4;
    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    std::array<ListGraph::Edge, 6> edges;

    edges[0] = g.addEdge(nodes[0], nodes[1]);
    edges[1] = g.addEdge(nodes[0], nodes[2]);
    edges[2] = g.addEdge(nodes[0], nodes[3]);
    edges[3] = g.addEdge(nodes[1], nodes[2]);
    edges[4] = g.addEdge(nodes[1], nodes[3]);
    edges[5] = g.addEdge(nodes[2], nodes[3]);


    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = 1.;
    firstWeights[edges[1]] = 1.;
    firstWeights[edges[2]] = 10.;
    firstWeights[edges[3]] = 10.;
    firstWeights[edges[4]] = 1.;
    firstWeights[edges[5]] = 1.;
    
    std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> scenarioSecondStageCosts;
    // scenarioSecondStageCosts.push_back(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&secondWeights1));
    
    // std::make_unique<lemon::ListGraph::EdgeMap<double>>
    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));
    // (*scenarioSecondStageCosts[0]).reset(&secondWeights1);

    (*scenarioSecondStageCosts[0])[edges[0]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[1]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[2]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[3]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[4]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[5]] = 1.5;
    

    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

    (*scenarioSecondStageCosts[1])[edges[0]] = 1.5;
    (*scenarioSecondStageCosts[1])[edges[1]] = 1.5;
    (*scenarioSecondStageCosts[1])[edges[2]] = 1.;
    (*scenarioSecondStageCosts[1])[edges[3]] = 2.;
    (*scenarioSecondStageCosts[1])[edges[4]] = 1.5;
    (*scenarioSecondStageCosts[1])[edges[5]] = 1.5;

    std::vector<double> probs{0.4, 0.6};
    auto res = bruteForceEnumeration(g, firstWeights, probs, scenarioSecondStageCosts);

    ASSERT_NEAR(res, 3, 0.00000001);
}

TEST(TwoStageSuite, Test6) {

    ListGraph g;
    const unsigned nodeNumber = 4;
    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    std::array<ListGraph::Edge, 6> edges;

    edges[0] = g.addEdge(nodes[0], nodes[1]);
    edges[1] = g.addEdge(nodes[0], nodes[2]);
    edges[2] = g.addEdge(nodes[0], nodes[3]);
    edges[3] = g.addEdge(nodes[1], nodes[2]);
    edges[4] = g.addEdge(nodes[1], nodes[3]);
    edges[5] = g.addEdge(nodes[2], nodes[3]);


    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = .5;
    firstWeights[edges[1]] = 1.;
    firstWeights[edges[2]] = 10.;
    firstWeights[edges[3]] = 10.;
    firstWeights[edges[4]] = 1.;
    firstWeights[edges[5]] = 1.;
    
    std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> scenarioSecondStageCosts;
    // scenarioSecondStageCosts.push_back(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&secondWeights1));
    
    // std::make_unique<lemon::ListGraph::EdgeMap<double>>
    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));
    // (*scenarioSecondStageCosts[0]).reset(&secondWeights1);

    (*scenarioSecondStageCosts[0])[edges[0]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[1]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[2]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[3]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[4]] = 1.5;
    (*scenarioSecondStageCosts[0])[edges[5]] = 1.5;
    

    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

    (*scenarioSecondStageCosts[1])[edges[0]] = 1.5;
    (*scenarioSecondStageCosts[1])[edges[1]] = 1.5;
    (*scenarioSecondStageCosts[1])[edges[2]] = 1.;
    (*scenarioSecondStageCosts[1])[edges[3]] = 2.;
    (*scenarioSecondStageCosts[1])[edges[4]] = 1.5;
    (*scenarioSecondStageCosts[1])[edges[5]] = 1.5;

    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

    (*scenarioSecondStageCosts[2])[edges[0]] = 0.5;
    (*scenarioSecondStageCosts[2])[edges[1]] = 1.5;
    (*scenarioSecondStageCosts[2])[edges[2]] = 1.5;
    (*scenarioSecondStageCosts[2])[edges[3]] = 0.5;
    (*scenarioSecondStageCosts[2])[edges[4]] = 1.;
    (*scenarioSecondStageCosts[2])[edges[5]] = 0.5;

    std::vector<double> probs{0.2, 0.2, 0.6};
    auto res = bruteForceEnumeration(g, firstWeights, probs, scenarioSecondStageCosts);

    ASSERT_NEAR(res, 2.2, 0.00000001);
}

TEST(TwoStageSuite, Test7) {

    ListGraph g;
    const unsigned nodeNumber = 4;
    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    std::array<ListGraph::Edge, 6> edges;

    edges[0] = g.addEdge(nodes[0], nodes[1]);
    edges[1] = g.addEdge(nodes[0], nodes[2]);
    edges[2] = g.addEdge(nodes[0], nodes[3]);
    edges[3] = g.addEdge(nodes[1], nodes[2]);
    edges[4] = g.addEdge(nodes[1], nodes[3]);
    edges[5] = g.addEdge(nodes[2], nodes[3]);


    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = 1.7;
    firstWeights[edges[1]] = .9;
    firstWeights[edges[2]] = 8.4;
    firstWeights[edges[3]] = 4.1;
    firstWeights[edges[4]] = 5.8;
    firstWeights[edges[5]] = 4.1;
    
    std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> scenarioSecondStageCosts;
    // scenarioSecondStageCosts.push_back(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&secondWeights1));
    
    // std::make_unique<lemon::ListGraph::EdgeMap<double>>
    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));
    // (*scenarioSecondStageCosts[0]).reset(&secondWeights1);

    (*scenarioSecondStageCosts[0])[edges[0]] = 3.7;
    (*scenarioSecondStageCosts[0])[edges[1]] = 2.9;
    (*scenarioSecondStageCosts[0])[edges[2]] = 3.;
    (*scenarioSecondStageCosts[0])[edges[3]] = 2.;
    (*scenarioSecondStageCosts[0])[edges[4]] = .4;
    (*scenarioSecondStageCosts[0])[edges[5]] = 2.1;
    

    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

    (*scenarioSecondStageCosts[1])[edges[0]] = 7.3;
    (*scenarioSecondStageCosts[1])[edges[1]] = 5.6;
    (*scenarioSecondStageCosts[1])[edges[2]] = 5.9;
    (*scenarioSecondStageCosts[1])[edges[3]] = 10.;
    (*scenarioSecondStageCosts[1])[edges[4]] = 3.1;
    (*scenarioSecondStageCosts[1])[edges[5]] = 7.8;

    scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

    (*scenarioSecondStageCosts[2])[edges[0]] = 9.4;
    (*scenarioSecondStageCosts[2])[edges[1]] = 5.1;
    (*scenarioSecondStageCosts[2])[edges[2]] = 7.6;
    (*scenarioSecondStageCosts[2])[edges[3]] = 6.7;
    (*scenarioSecondStageCosts[2])[edges[4]] = 0.2;
    (*scenarioSecondStageCosts[2])[edges[5]] = 4.6;

    std::vector<double> probs{0.397, 0.039, 0.564};
    auto res = bruteForceEnumeration(g, firstWeights, probs, scenarioSecondStageCosts);

    ASSERT_NEAR(res, 2.9925, 0.00000001);
}


TEST(UtilitiesSuite, Test2) {

    ListGraph g;
    const unsigned nodeNumber = 4;
    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    std::array<ListGraph::Edge, 6> edges;

    edges[0] = g.addEdge(nodes[0], nodes[1]);
    edges[1] = g.addEdge(nodes[0], nodes[2]);
    edges[2] = g.addEdge(nodes[0], nodes[3]);
    edges[3] = g.addEdge(nodes[1], nodes[2]);
    edges[4] = g.addEdge(nodes[1], nodes[3]);
    edges[5] = g.addEdge(nodes[2], nodes[3]);


    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = 1.;
    firstWeights[edges[1]] = 1.;
    firstWeights[edges[2]] = 1.;
    firstWeights[edges[3]] = 1.;
    firstWeights[edges[4]] = 1.;
    firstWeights[edges[5]] = 1.;

    lemon::ListGraph::EdgeMap<std::vector<double>> edgeMap(g);
    std::vector<double> scenarioProbabilities;

    edgeWeightIncrease(edgeMap, scenarioProbabilities, g, firstWeights, 1., 2., 10, rng);

    for (int i=0; i<scenarioProbabilities.size(); i++) {

        for (lemon::ListGraph::EdgeIt e(g); e!=lemon::INVALID; ++e) {
        
            // std::cout << edgeMap[e][i] << ", ";
            EXPECT_NEAR(edgeMap[e][i],2, 0.0000000001);
        }
        // std::cout << "\t with prob." << scenarioProbabilities[i] << '\n';
    }
}

TEST(UtilitiesSuite, Test3) {

    ListGraph g;
    const unsigned nodeNumber = 4;
    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    std::array<ListGraph::Edge, 6> edges;

    edges[0] = g.addEdge(nodes[0], nodes[1]);
    edges[1] = g.addEdge(nodes[0], nodes[2]);
    edges[2] = g.addEdge(nodes[0], nodes[3]);
    edges[3] = g.addEdge(nodes[1], nodes[2]);
    edges[4] = g.addEdge(nodes[1], nodes[3]);
    edges[5] = g.addEdge(nodes[2], nodes[3]);

    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = 1.;
    firstWeights[edges[1]] = 1.;
    firstWeights[edges[2]] = 1.;
    firstWeights[edges[3]] = 1.;
    firstWeights[edges[4]] = 1.;
    firstWeights[edges[5]] = 1.;

    lemon::ListGraph::EdgeMap<std::vector<double>> edgeMap(g);
    std::vector<double> scenarioProbabilities;

    edgeWeightIncrease(edgeMap, scenarioProbabilities, g, firstWeights, 0., 2., 10, rng);

    for (int i=0; i<scenarioProbabilities.size(); i++) {

        for (lemon::ListGraph::EdgeIt e(g); e!=lemon::INVALID; ++e) {
        
            // std::cout << edgeMap[e][i] << ", ";
            EXPECT_NEAR(edgeMap[e][i],1, 0.0000000001);
        }
        // std::cout << "\t with prob." << scenarioProbabilities[i] << '\n';
    }
}

TEST(UtilitiesSuite, Test4) {

    ListGraph g;
    const unsigned nodeNumber = 4;
    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    std::array<ListGraph::Edge, 4> edges;

    edges[0] = g.addEdge(nodes[0], nodes[1]);
    edges[1] = g.addEdge(nodes[0], nodes[2]);
    edges[2] = g.addEdge(nodes[1], nodes[3]);
    edges[3] = g.addEdge(nodes[2], nodes[3]);

    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = 1.;
    firstWeights[edges[1]] = 1.;
    firstWeights[edges[2]] = 1.;
    firstWeights[edges[3]] = 1.;

    lemon::ListGraph::EdgeMap<std::vector<double>> edgeMap(g);
    std::vector<double> scenarioProbabilities;

    edgeWeightIncrease(edgeMap, scenarioProbabilities, g, firstWeights, 0.5, 2., 200000, rng);
    std::cout << "DIESER TEST KANN MIT GERINGER WAHRSCHEINLICHKEIT FEHLSCHLAGEN!\n";

    for (int i=0; i<scenarioProbabilities.size(); i++) {
        
        EXPECT_NEAR(scenarioProbabilities[i],0.0625, 0.002);
        

        for (lemon::ListGraph::EdgeIt e(g); e!=lemon::INVALID; ++e) {
            std::cout << edgeMap[e][i] << ", ";
        }
        std::cout << "\t with prob." << scenarioProbabilities[i] << '\n';

    }
    // EXPECT_EQ(1,2);
}

TEST(TwoStageSuite, Test8) {

    ListGraph g;
    const unsigned nodeNumber = 4;
    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    std::array<ListGraph::Edge, 6> edges;

    edges[0] = g.addEdge(nodes[0], nodes[1]);
    edges[1] = g.addEdge(nodes[0], nodes[2]);
    edges[2] = g.addEdge(nodes[0], nodes[3]);
    edges[3] = g.addEdge(nodes[1], nodes[2]);
    edges[4] = g.addEdge(nodes[1], nodes[3]);
    edges[5] = g.addEdge(nodes[2], nodes[3]);


    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = 1.;
    firstWeights[edges[1]] = 1.;
    firstWeights[edges[2]] = 1.;
    firstWeights[edges[3]] = 1.;
    firstWeights[edges[4]] = 1.;
    firstWeights[edges[5]] = 1.;
    
    lemon::ListGraph::EdgeMap<std::vector<double>> edgeMap(g);
    std::vector<double> scenarioProbabilities;

    edgeWeightIncrease(edgeMap, scenarioProbabilities, g, firstWeights, 0.0, 2., 20, rng);
   
    auto res = bruteForceEnumeration(g, firstWeights, scenarioProbabilities, edgeMap);

    ASSERT_NEAR(res, 3., 0.00000001);
}

TEST(TwoStageSuite, Test9) {

    ListGraph g;
    const unsigned nodeNumber = 4;
    std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes[i] = g.addNode();
    }

    std::array<ListGraph::Edge, 6> edges;

    edges[0] = g.addEdge(nodes[0], nodes[1]);
    edges[1] = g.addEdge(nodes[0], nodes[2]);
    edges[2] = g.addEdge(nodes[0], nodes[3]);
    edges[3] = g.addEdge(nodes[1], nodes[2]);
    edges[4] = g.addEdge(nodes[1], nodes[3]);
    edges[5] = g.addEdge(nodes[2], nodes[3]);


    ListGraph::EdgeMap<double> firstWeights(g); 
    firstWeights[edges[0]] = 1.;
    firstWeights[edges[1]] = 1.;
    firstWeights[edges[2]] = 1.;
    firstWeights[edges[3]] = 1.;
    firstWeights[edges[4]] = 1.;
    firstWeights[edges[5]] = 1.;
    
    lemon::ListGraph::EdgeMap<std::vector<double>> edgeMap(g);
    std::vector<double> scenarioProbabilities;

    edgeWeightIncrease(edgeMap, scenarioProbabilities, g, firstWeights, 0.5, 2., 20000, rng);
   
    auto res = bruteForceEnumeration(g, firstWeights, scenarioProbabilities, edgeMap);

    ASSERT_NEAR(res, 3., 0.00000001);
}