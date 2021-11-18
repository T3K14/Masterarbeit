#include <gtest/gtest.h>
#include <array>
#include <vector>
#include <iostream>
#include <fstream>

#include "../two_stage_gurobi.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

/* hier sollen die gurobi und die approximationsmethoden getestet werden */


    // test without valid LP-Solution, only to test basic functionality
    // visualisierung des Problems befindet sich in meinen Notizen vom 09.10.21

using namespace std;

// Hier soll der approxalg auf die simplen Tests (die einfachen, wo zB. in Phase 1 alles billiger ist etc.) angewendet werden
// gibt nur das Problem, dass der Approxalg unterschiedliche Ergebnisse ausgeben kann
TEST(TrivialSuite, Test1) {

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

    // alle Kanten werden nur teurer, also idealerweise alle am Anfang kaufen
    double res = solve_relaxed_lp(mst);
    mst.save_lp_result_map("lp_test_trivial1");

    // ASSERT_NEAR(.5, res, 0.0000001);

    // lemon::ListGraph::EdgeMap<double> output(mst.g);
    mst.approximate(rng);
    mst.save_approx_result_map("approx_test_trivial1");

    string filepath = R"(/gss/work/xees8992/trivial1.txt)";
    ofstream outFile(filepath, ios_base::app);

    if (outFile.is_open()) {
        outFile << expected_costs << "\n";
        outFile.close();
    }
    else {
        cout << "Error beim Fileoeffnen\n";
    }

}

// Tests im LPSuite testen, ob der LP-Teil mit dem gurobi-Algorithmus verlaessliche Ergebnisse liefert
TEST(LPSuite, TrivialTest) {

    try {
        // test 1
        unsigned int numberScenarios = 1;
        unsigned int numberNodes = 2;

        std::vector<double> scenarioProbabilities {1.0};
        std::vector<double> firstStageWeights {1.5};
        std::vector<std::vector<double>> secondStageWeights {{{0.5}}};

        FullyConnectedTwoStageMST mst(numberNodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

        double res = solve_relaxed_lp(mst);
        mst.save_lp_result_map("lp_test_two_nodes");

        ASSERT_NEAR(.5, res, 0.0000001);

        // lemon::ListGraph::EdgeMap<double> output(mst.g);
        mst.approximate(rng);
        mst.save_approx_result_map("approx_test_two_nodes");


        double expected_costs = mst.calculate_expected_from_bool_map(mst.approx_first_stage_map);
        //output, zum Vergleichen, da der Test nur auf dem Cluster laeuft, schreibe ich hier direkt den Work-path rein

        string filepath = R"(/gss/work/xees8992/trivialtest.txt)";
        ofstream outFile(filepath, ios_base::app);

        if (outFile.is_open()) {
            outFile << expected_costs << "\n";
            outFile.close();
        }
        else {
            cout << "Error beim Fileoeffnen\n";
        }

        // cout << expected_costs << endl;

    }
    catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } 
    catch(...) {
        cout << "Exception during optimization" << endl;
    }
}

TEST(LPSuite, Test2) {

    try {
        // test 2
        unsigned int numberScenarios = 1;
        unsigned int numberNodes = 3;

        std::vector<double> scenarioProbabilities {1.0};
        std::vector<double> firstStageWeights {1.0, 1.0, 1.0};
        std::vector<std::vector<double>> secondStageWeights {{{0.5, 1.0, 1.0}}};

        FullyConnectedTwoStageMST mst(numberNodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

        double res = solve_relaxed_lp(mst);
        mst.save_lp_result_map("lp_test_three_nodes");

        // 1.25, weil man fuer alle Cuts immer zwei Kanten hat und daher muessen alle mindestens einmal den Wert 0.5 zugeordnet bekommen, da die erste Kante in stage 2 aber mit 0.5
        // multipliziert wird, lohnt es sich mehr die in Phase 2 zu kaufen und zu den 2 mal 0.5 kommt noch 0.5*0.5 dazu 
        ASSERT_NEAR(1.25, res, 0.0000001);
        mst.approximate(rng);
        mst.save_approx_result_map("approx_test_three_nodes");

        double expected_costs = mst.calculate_expected_from_bool_map(mst.approx_first_stage_map);
        //output, zum Vergleichen
        string filepath = R"(/gss/work/xees8992/test2.txt)";
        ofstream outFile(filepath, ios_base::app);

        if (outFile.is_open()) {
            outFile << expected_costs << "\n";
            outFile.close();
        }
        else {
            cout << "Error beim Fileoeffnen\n";
        }
    }
    catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } 
    catch(...) {
        cout << "Exception during optimization" << endl;
    }
}

TEST(LPSuite, Test3) {

    try {
        // test 2
        unsigned int numberScenarios = 2;
        unsigned int numberNodes = 3;

        std::vector<double> scenarioProbabilities {0.9, 0.1};
        std::vector<double> firstStageWeights {1.0, 1.0, 1.0};
        std::vector<std::vector<double>> secondStageWeights {{{0.5, 1.0, 1.0}, {2.99, 1.5, 1.5}}};

        FullyConnectedTwoStageMST mst(numberNodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

        double res = solve_relaxed_lp(mst);
        mst.save_lp_result_map("lp_test_three_nodes2");

        // dazu auch aufzeichnungen vom 26.10.21
        // es ist so, dass fuer das zweite Szenario so lange Kante 1 gekauft wird, bis deren Kosten die von den anderen beiden in diesem Szenario uebersteigen (hier 3), da sie nur 2.99
        // kostet, wird sie hier noch gekauft
        ASSERT_NEAR(1.3745, res, 0.0000001);

        mst.approximate(rng);
        mst.save_approx_result_map("approx_test_three_nodes2");

        double expected_costs = mst.calculate_expected_from_bool_map(mst.approx_first_stage_map);
        //output, zum Vergleichen
        string filepath = R"(/gss/work/xees8992/test3.txt)";
        ofstream outFile(filepath, ios_base::app);

        if (outFile.is_open()) {
            outFile << expected_costs << "\n";
            outFile.close();
        }
        else {
            cout << "Error beim Fileoeffnen\n";
        }    
    }
    catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } 
    catch(...) {
        cout << "Exception during optimization" << endl;
    }
}

//         //test 2

//         unsigned int numberScenarios = 3;
//         unsigned int numberNodes = 4;

//     // std::array<double, 3> scenarioProbabilities {0.2, 0.2, 0.6};
//     // std::array<double, 6> firstStageWeights {0.5, 1., 10., 10., 1., 1.};
//     // std::array<std::array<double, 6>, 3> secondStageWeights = {{{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}, {1.5, 1.5, 1., 2., 1.5, 1.5}, {0.5, 1.5, 1.5, 0.5, 1., 0.5}}};
    
//         std::vector<double> scenarioProbabilities {0.2, 0.2, 0.6};
//         std::vector<double> firstStageWeights {0.5, 1., 10., 10., 1., 1.};
//         std::vector<std::vector<double>> secondStageWeights {{{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}, {1.5, 1.5, 1., 2., 1.5, 1.5}, {0.5, 1.5, 1.5, 0.5, 1., 0.5}}};


//         FullyConnectedTwoStageMST mst(numberNodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

//         //std::cout << "Komme ich bis hier?\n";

//         solve_relaxed_lp(mst);
//         mst.save_lp_result_map("second");
//     }

//     catch(GRBException e) {
//         cout << "Error code = " << e.getErrorCode() << endl;
//         cout << e.getMessage() << endl;
//     } 
//     catch(...) {
//         cout << "Exception during optimization" << endl;
//     }

//     return 0;

// //    ASSERT_NEAR(1.0, 1.0, 0.0000001);
// }
