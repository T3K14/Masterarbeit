#include <gtest/gtest.h>
#include <array>
#include <vector>

#include "../two_stage.hpp"
#include "../utilities.hpp"

/* hier sollen die gurobi und die approximationsmethoden getestet werden */

TEST(ApproxSuite, TrivialTest) {

    // test without valid LP-Solution, only to test basic functionality
    // visualisierung des Problems befindet sich in meinen Notizen vom 09.10.21
    unsigned int numberScenarios = 3;
    unsigned int numberNodes = 4;

    // std::array<double, 3> scenarioProbabilities {0.2, 0.2, 0.6};
    // std::array<double, 6> firstStageWeights {0.5, 1., 10., 10., 1., 1.};
    // std::array<std::array<double, 6>, 3> secondStageWeights = {{{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}, {1.5, 1.5, 1., 2., 1.5, 1.5}, {0.5, 1.5, 1.5, 0.5, 1., 0.5}}};
    
    std::vector<double> scenarioProbabilities {0.2, 0.2, 0.6};
    std::vector<double> firstStageWeights {0.5, 1., 10., 10., 1., 1.};
    std::vector<std::vector<double>> secondStageWeights {{{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}, {1.5, 1.5, 1., 2., 1.5, 1.5}, {0.5, 1.5, 1.5, 0.5, 1., 0.5}}};

    FullyConnectedTwoStageMST mst(numberNodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

    solve_relaxed_lp(mst);

    ASSERT_NEAR(1.0, 1.0, 0.0000001);

}