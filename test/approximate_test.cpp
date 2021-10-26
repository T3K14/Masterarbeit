#include <gtest/gtest.h>
#include <array>
#include <vector>
#include <iostream>

#include "../two_stage_gurobi.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

/* hier sollen die gurobi und die approximationsmethoden getestet werden */

//TEST(ApproxSuite, TrivialTest) {

    // test without valid LP-Solution, only to test basic functionality
    // visualisierung des Problems befindet sich in meinen Notizen vom 09.10.21

using namespace std;

int main() {

    try {
        // test 1
        unsigned int numberScenarios0 = 1;
        unsigned int numberNodes0 = 2;

    // std::array<double, 3> scenarioProbabilities {0.2, 0.2, 0.6};
    // std::array<double, 6> firstStageWeights {0.5, 1., 10., 10., 1., 1.};
    // std::array<std::array<double, 6>, 3> secondStageWeights = {{{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}, {1.5, 1.5, 1., 2., 1.5, 1.5}, {0.5, 1.5, 1.5, 0.5, 1., 0.5}}};
    
        std::vector<double> scenarioProbabilities0 {1.0};
        std::vector<double> firstStageWeights0 {1.5};
        std::vector<std::vector<double>> secondStageWeights0 {{{0.5}}};

        FullyConnectedTwoStageMST mst0(numberNodes0, firstStageWeights0, secondStageWeights0, scenarioProbabilities0);

        //std::cout << "Komme ich bis hier?\n";

        solve_relaxed_lp(mst0);
        mst0.save_lp_result_map("first");



        //test 2

        unsigned int numberScenarios = 3;
        unsigned int numberNodes = 4;

    // std::array<double, 3> scenarioProbabilities {0.2, 0.2, 0.6};
    // std::array<double, 6> firstStageWeights {0.5, 1., 10., 10., 1., 1.};
    // std::array<std::array<double, 6>, 3> secondStageWeights = {{{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}, {1.5, 1.5, 1., 2., 1.5, 1.5}, {0.5, 1.5, 1.5, 0.5, 1., 0.5}}};
    
        std::vector<double> scenarioProbabilities {0.2, 0.2, 0.6};
        std::vector<double> firstStageWeights {0.5, 1., 10., 10., 1., 1.};
        std::vector<std::vector<double>> secondStageWeights {{{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}, {1.5, 1.5, 1., 2., 1.5, 1.5}, {0.5, 1.5, 1.5, 0.5, 1., 0.5}}};


        FullyConnectedTwoStageMST mst(numberNodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

        //std::cout << "Komme ich bis hier?\n";

        solve_relaxed_lp(mst);
        mst.save_lp_result_map("second");
    }

    catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } 
    catch(...) {
        cout << "Exception during optimization" << endl;
    }

    return 0;

//    ASSERT_NEAR(1.0, 1.0, 0.0000001);
}
