#include <gtest/gtest.h>

#include "../simulate.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

using namespace lemon;

TEST(RSBSuite, Test1) {

    int number_scenarios = 5;

    // formuliere Problemstellung

    NRandomScenarioCreator sc(number_scenarios, rng);    
    double c = 4.;
    double p = 0.6;
    int n = 50;

    KantenFaktorCreator2 kfc2(p, 2., rng);
    TreePlusC ensemble2(n, sc, kfc2, rng, c);
    ensemble2.initialize();


    

    // ASSERT_EQ(resInt, 6);
}