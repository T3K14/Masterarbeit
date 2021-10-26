#include <gtest/gtest.h>
#include <array>
#include <vector>
#include <iostream>

#include "../two_stage_gurobi.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

/* hier sollen die gurobi und die approximationsmethoden getestet werden */


    // test without valid LP-Solution, only to test basic functionality
    // visualisierung des Problems befindet sich in meinen Notizen vom 09.10.21

using namespace std;

TEST(ApproxSuite, TrivialTest) {

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

    }
    catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } 
    catch(...) {
        cout << "Exception during optimization" << endl;
    }
}

TEST(ApproxSuite, Test2) {

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

        // 1.25, weil man fuer alle Cuts immer zwei Kanten hat und daher muessen alle mindestens einmal den Wert 0.5 zugeordnet bekommen, da die erste Kante im stage 2 aber mit 0.5
        // multipliziert wird, lohnt es sich mehr die in Phase 2 zu kaufen und zu den 2 mal 0.5 kommt noch 0.5*0.5 dazu 
        ASSERT_NEAR(1.25, res, 0.0000001);
        mst.approximate(rng);
        mst.save_approx_result_map("approx_test_three_nodes");
    }
    catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } 
    catch(...) {
        cout << "Exception during optimization" << endl;
    }
}

TEST(ApproxSuite, Test3) {

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
