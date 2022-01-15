#include <gtest/gtest.h>

#include "../simulate.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

using namespace lemon;

TEST(SimulateSuite, Test1) {

    Tree tree();

    simulate(1000, tree, "4bvsApprox");
}