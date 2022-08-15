#include "simulate.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"

#include <iostream>

int main(int argc, char * argv[]) {


    bool tracking = false;

    int number_scenarios = 5;

    // formuliere Problemstellung

    NRandomScenarioCreator sc(number_scenarios, rng);    
    double c = 4.;

    KantenFaktorCreator2 kfc2(p, 2., rng);
    TreePlusC ensemble2(n, sc, kfc2, rng, c);
    ensemble2.initialize();

    // loese optimal und speichere mir die Ergebnisse, die ich fuer spaetere Auswertung benoetige (das Ergebnis und die Map, die die Erstphasenauswahl beinhaltet)
    double res_optimum = ensemble.optimum(tracking, tracking_path);

    ensemble.disturb();

    // veraendere die Problemstellung

    // loese erneut optimal

    // speichere Ergebnisse ab

    ensemble.two_stage_problem.save_result_map(ensemble.two_stage_problem.optimum_first_stage_map, map_path);



    return 0;
}