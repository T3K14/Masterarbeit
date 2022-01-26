#include <gtest/gtest.h>

#include "../simulate.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

using namespace lemon;

TEST(SimulateSuite, Test1) {

    NRandomScenarioCreator sc(3, rng);
    RandomTestCreator ec(0., 10., rng);

    Tree tree(10, sc, ec, rng);

    
}