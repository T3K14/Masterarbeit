#include <gtest/gtest.h>

#include "../simulate.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

using namespace lemon;

TEST(RSBSuite, Test1) {

    int number_scenarios = 5;

    // formuliere Problemstellung

    NRandomScenarioCreator sc(number_scenarios, rng);    
    double c = 2.5;
    double p = 0.4;
    int n = 4;

    // KantenFaktorCreator2 kfc2(p, 2., rng);
    RandomTestCreator rtc(0., 10., rng);
    TreePlusC ensemble(n, sc, rtc, rng, c);

    rng.seed(1000);

    ensemble.initialize();

    boost_path rsb_path("/gss/work/xees8992/rsb_test");

    // speichere Problemstellung
    ensemble.save_current_graph(rsb_path, "graph");
    ensemble.save_current_scenarios(rsb_path, "");

    auto opt = ensemble.optimum(false, boost_path());
    auto opt_b = ensemble.bruteforce(false, boost_path());

    ensemble.two_stage_problem.save_result_map(ensemble.two_stage_problem.optimum_first_stage_map, rsb_path / "vorher.txt");

    std::cout << "Optimum : " << opt << std::endl;

    // nochmal schauen, ob fuer beide Versionen die selben Ergebnisse rauskommen (optimum soll nur schneller als bruteforce sein, aber die selben Ergebnisse liefern)
    ASSERT_NEAR(opt, opt_b, 0.0000001);
    ASSERT_NEAR(opt, 5.5869, 0.00001);      // das habe ich nachgerechnet und es ist die beste Loesung fuer dieses Problem!

    ensemble.disturb(Stoerung::SingleIncrease);

    // rufe optimum-Fkt erneut auf
    auto opt2 = ensemble.optimum(false, boost_path());
    std::cout << "Optimum2 : " << opt2 << std::endl;

    ASSERT_NEAR(opt2, 8.33692, 0.00001);    // das habe ich nachgerechnet und es ist die beste Loesung fuer das gestoerte Problem

    ensemble.two_stage_problem.save_result_map(ensemble.two_stage_problem.optimum_first_stage_map, rsb_path / "nachher.txt");

}