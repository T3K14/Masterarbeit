#include <gtest/gtest.h>
#include <array>
#include <vector>

#include "../two_stage.hpp"
#include "../utilities.hpp"

TEST(UtilitiesSuite, Test1) {

    // test, if the probabilities sum up to almost 1
    auto resVec = calcScenarioProbabilities(8000, rng);

    double sum = 0;
    for (double d : resVec) {
        sum += d;
    }

    ASSERT_NEAR(1.0, sum, 0.0000001);



}

