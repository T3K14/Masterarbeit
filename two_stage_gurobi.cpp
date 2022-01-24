#include "two_stage_gurobi.hpp"
#include "two_stage.hpp"
#include <lemon/hao_orlin.h>
#include <algorithm>
#include <iostream>

#include <chrono>

// nimmt ein two_stage_problem und loesst das mit hilfe von Gurobi
double solve_relaxed_lp(TwoStageProblem & two_stage_problem) {
    
    // Setup: jede Kante bekommt ein Array von Gurobi-Variablen, eine fuer Stage 1 und jeweils eine fuer jedes Szenario in Stage 2
    GRBEnv env = GRBEnv();

    GRBModel model = GRBModel(env);
    model.set(GRB_IntParam_OutputFlag, 0);

    // jeder Kante wird ein Array von GurobiVariablen zugeordnet
    lemon::ListGraph::EdgeMap<GRBVar *> gurobi_variables_map(two_stage_problem.g);

    // das wird die objective function 
    GRBLinExpr obj = 0;

    // ich tue so, als haette ich die edges nicht zwangsweise selbst in einem array sondern nutze den lemon edge iterator
    for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
        
        // jede Kante bekommt array mit Variablen fuer alle Szenarien (+1 fuer die erste Stage)
        gurobi_variables_map[e] = model.addVars(two_stage_problem.numberScenarios + 1, GRB_CONTINUOUS);               // werden unten gefreet

        // und ich kann gleich schon die objektive function mit aufbauen
        // stage 1
        obj += two_stage_problem.firstStageWeights[e] * gurobi_variables_map[e][0];

        // stage 2
        // der index geht bei 0 los, aber von den gurobi_variablen muss ich immer 1 drauf rechnen, weil die nullte variable fuer der erste Stage ist
        for (int i=0; i<two_stage_problem.numberScenarios; i++) {
            obj += two_stage_problem.secondStageProbabilities[i] * two_stage_problem.secondStageWeights[e][i] * gurobi_variables_map[e][i+1];
        }
    }

    model.setObjective(obj, GRB_MINIMIZE);

    // Ende vom Setup
    // jetzt so lange Cut-Constraints hinzufuegen, bis die Bedingungen immer erfuellt sind 
    while(true) {

        model.optimize();

        double min_cut_value;

        // map der capacities (wird fuer jedes szenario neu beschrieben), brauche ich fuer den HaoOrlin-Algorithmus
        lemon::ListGraph::EdgeMap<double> capacity_map(two_stage_problem.g);

        // map in der angegeben wird, welche Knoten in der einen MinCut-Teilmenge drin sind, output des HaoOrlin-Algorithmus, aus dem ich dann die Kanten bestimmen kann, die die Cut-
        // Teilmengen verbinden
        lemon::ListGraph::NodeMap<bool> min_cut_result_map(two_stage_problem.g);

        // gehe alle szenarien durch und suche nach mincut, der die Bedingung nicht erfuellt
        for (int i=1; i<two_stage_problem.numberScenarios+1; i++) {     //index geht bei 1 los, weil er nur benutzt wird, um auf die gurobi_variablen zuzugreifen und die haben zum index 0 den Variable fuer die erste stage 

            // die x^_e und x^i_e Werte addieren und als capacity Werte eintragen
            for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
                // die capacity ist die summe aus den Werten der ersten Phase und den der i-ten Phase
                //capacity_map[edges[j]] = variables_array[j].get(GRB_DoubleAttr) + varables_array[i * numberEdges + j].get(GRB_DoubleAttr);
                capacity_map[e] = gurobi_variables_map[e][0].get(GRB_DoubleAttr_X) + gurobi_variables_map[e][i].get(GRB_DoubleAttr_X);
            }

            // suche minCut
            lemon::HaoOrlin<lemon::ListGraph, lemon::ListGraph::EdgeMap<double>> hao(two_stage_problem.g, capacity_map);
            hao.init();
            hao.calculateIn();

            // falls der minCut die Bedingung verletzt, der Aufruf speichert direkt auch die bools fuer die Teilmengen in min_cut_result_map
            min_cut_value = hao.minCutMap(min_cut_result_map);
            if(min_cut_value < 1) {
                
                // fuege neues constraint hinzu, damit diese Bedingung in zukunft erfuellt ist
                GRBLinExpr constraint = 0.0;

                // dazu muss ich erstmal alle Kanten ermitteln, die bei diesem minCut die beiden Teilmengen verbinden
                for (lemon::ListGraph::NodeIt n(two_stage_problem.g); n != lemon::INVALID; ++n) {
                    if (min_cut_result_map[n]) {
                        for (lemon::ListGraph::IncEdgeIt e(two_stage_problem.g, n); e != lemon::INVALID; ++e) {
                            // wenn entweder n true und die andere edge false ist oder andersrum
                            if ((min_cut_result_map[two_stage_problem.g.source(e)] && !min_cut_result_map[two_stage_problem.g.target(e)]) || (!min_cut_result_map[two_stage_problem.g.source(e)] && min_cut_result_map[two_stage_problem.g.target(e)])) {

                                constraint += gurobi_variables_map[e][0] + gurobi_variables_map[e][i];

                            }
                        }
                    }
                }

                // constraint jetzt noch hinzufuegen
                model.addConstr(constraint, GRB_GREATER_EQUAL, 1.0);

                // und ich gehe aus derm for-loop raus, in der Annahme, dass allein diese Veraenderung schon was bewirkt und es sich vielleicht nicht lohnt, noch weiter durch alle anderen
                //Szenarien zu schauen
                break;
            } 

        }
        // hier hab ich entweder alle szenarien durch und nicht gebreakt, was bedeutet, dass alle min-cut-values >= 1 sind oder ich bin entweder aus dem for-loop rausgebreakt, also
        // muss ich nochmal checken, ob auch der letzte Wert >= 1 ist, weil wenn nicht, muss ich neu mit der neuen Bed. optimieren

        // falls an diesem Punkt der minCut das Constraint erfuelt, gibt es kein Szenario mehr, wo der minCut gegen das Constraint verstoest und ich bin fertig mit der LP-Loesung
        if(min_cut_value >= 1) {
            break;
        }
        // ansonsten optimiere erneut
    }

    double res = model.get(GRB_DoubleAttr_ObjVal);              // res ist der Wert der objective function
    //double res = obj.getValue();
    
    // ich schreibe nun in die uebergebene EdgeMap die Ergebnisse der optimierten LP-Variablen und free die hier allocateten Variablen arrays
    for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {

        // std::copy_n(model.get(GRB_DoubleAttr_X, gurobi_variables_map[e], two_stage_problem.numberScenarios+1), two_stage_problem.lp_results_map[e].size(), two_stage_problem.lp_results_map[e].begin());

        double * lp_edge_solutions = model.get(GRB_DoubleAttr_X, gurobi_variables_map[e], two_stage_problem.numberScenarios+1);
        for (int i=0; i<two_stage_problem.numberScenarios+1; i++) {
            two_stage_problem.lp_results_map[e].push_back(lp_edge_solutions[i]);
        }

        // hier free ich die Variablen arrays pro Kante
        delete[] gurobi_variables_map[e];

    }

    // for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        // delete[] gurobi_variables_map[e];
    // }

    //vielleicht brauche ich auch gar nicht die Variablen zurueckgeben, sondern nur deren Werte

    // return gurobi_variables_map;

    return res;
}

// nimmt ein two_stage_problem und loesst das mit hilfe von Gurobi
double solve_relaxed_lp(TwoStageProblem & two_stage_problem, unsigned long & counter, std::chrono::seconds & setup_zeit, std::chrono::seconds & loop_zeit, std::vector<std::chrono::milliseconds> & opti_times) { //, lemon::ListGraph::EdgeMap<std::vector<double>> & two_stage_problem.lp_results_map) {

    auto t_start_setup = std::chrono::high_resolution_clock::now();


    /* DEBUG
    // std::cout << "pkt 1 in Funktion\n";
    // auto eddy = two_stage_problem.g.edgeFromId(0);

    // std::cout << "prob:" << two_stage_problem.secondStageProbabilities[0] << "\n";
    // std::cout << "first:" << two_stage_problem.firstStageWeights[eddy] << "\n";
    // std::cout << "second:" << two_stage_problem.secondStageWeights[eddy][0] << "\n";
    ENDE DEBUG */

    // Setup: jede Kante bekommt ein Array von Gurobi-Variablen, eine fuer Stage 1 und jeweils eine fuer jedes Szenario in Stage 2
    GRBEnv env = GRBEnv();

    GRBModel model = GRBModel(env);
    model.set(GRB_IntParam_OutputFlag, 0);

    // jeder Kante wird ein Array von GurobiVariablen zugeordnet
    lemon::ListGraph::EdgeMap<GRBVar *> gurobi_variables_map(two_stage_problem.g);

    // das wird die objective function 
    GRBLinExpr obj = 0;

    // ich tue so, als haette ich die edges nicht zwangsweise selbst in einem array sondern nutze den lemon edge iterator
    for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
        
        // jede Kante bekommt array mit Variablen fuer alle Szenarien (+1 fuer die erste Stage)
        gurobi_variables_map[e] = model.addVars(two_stage_problem.numberScenarios + 1, GRB_CONTINUOUS);               // werden unten gefreet

        // und ich kann gleich schon die objektive function mit aufbauen
        // stage 1
        obj += two_stage_problem.firstStageWeights[e] * gurobi_variables_map[e][0];

        // stage 2
        // der index geht bei 0 los, aber von den gurobi_variablen muss ich immer 1 drauf rechnen, weil die nullte variable fuer der erste Stage ist
        for (int i=0; i<two_stage_problem.numberScenarios; i++) {
            obj += two_stage_problem.secondStageProbabilities[i] * two_stage_problem.secondStageWeights[e][i] * gurobi_variables_map[e][i+1];
        }
    }

    /* DEBUG
    // std::cout << "Pkt 2 in Funktion\n";
    // std::cout << obj.size() << std::endl; 
    // std::cout << obj.getCoeff(0) << ", und " << obj.getCoeff(1) << std::endl;
    ENDE DEBUG */

    model.setObjective(obj, GRB_MINIMIZE);


    // Ende vom Setup

    // TIMER bis hierhin und ab hier timen und Iterationen zaehlen

    auto t_end_setup = std::chrono::high_resolution_clock::now();
    setup_zeit = std::chrono::duration_cast<std::chrono::seconds>(t_end_setup - t_start_setup);

    //std::cout << "Setup-Zeit: " << setup_int.count() << "s\n";

    auto t_start_loop = std::chrono::high_resolution_clock::now();


    // unsigned long counter = 0;

    // vector, in den ich die Zeiten reinpacke, wie lange es gedauert hat, einmal zu optimieren.
    //std::vector<std::chrono::milliseconds> opti_times;

    // ENDE TIMER

    // jetzt so lange Cut-Constraints hinzufuegen, bis die Bedingungen immer erfuellt sind 
    while(true) {

        // COUNTER zum zaehlen, wie viele Iterationen hier benoetigt werden
        counter++;
        // wenn der counter zu groess wird, dann muss ich aufpassen, dass kein overvlow stattfindet

        if (counter > 18446744073709551613) {       // diese Zahl +2 ist die obere Grenze vom 8 byte unsigned long
            std::cout << "ACHTUNG: integer Overflow findet statt!\n";
        }
        // ENDE COUNTER


        // will auch tracken, wie lange die optimierungsschritte dauern
        auto t_start_opt = std::chrono::high_resolution_clock::now();

        model.optimize();

        // die folgenden zwei zeilen gehoeren auch noch dazu
        auto t_end_opt = std::chrono::high_resolution_clock::now();
        opti_times.push_back(std::chrono::duration_cast<std::chrono::seconds>(t_end_setup - t_start_setup));

        // DEBUG
        // std::cout << "\n\n Hier nach einem optimierungsvorgang\n";
        // // gebe wert der ersten variablen aus und der objective function
        // std::cout <<  "Objective:" << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
        // std::cout << gurobi_variables_map[two_stage_problem.g.edgeFromId(0)][0].get(GRB_DoubleAttr_X) << std::endl;
        // std::cout << gurobi_variables_map[two_stage_problem.g.edgeFromId(0)][1].get(GRB_DoubleAttr_X) << std::endl;
        // ENDE DEBUG

        double min_cut_value;

        // map der capacities (wird fuer jedes szenario neu beschrieben), brauche ich fuer den HaoOrlin-Algorithmus
        lemon::ListGraph::EdgeMap<double> capacity_map(two_stage_problem.g);

        // map in der angegeben wird, welche Knoten in der einen MinCut-Teilmenge drin sind, output des HaoOrlin-Algorithmus, aus dem ich dann die Kanten bestimmen kann, die die Cut-
        // Teilmengen verbinden
        lemon::ListGraph::NodeMap<bool> min_cut_result_map(two_stage_problem.g);

        // gehe alle szenarien durch und suche nach mincut, der die Bedingung nicht erfuellt
        for (int i=1; i<two_stage_problem.numberScenarios+1; i++) {     //index geht bei 1 los, weil er nur benutzt wird, um auf die gurobi_variablen zuzugreifen und die haben zum index 0 den Variable fuer die erste stage 

            // die x^_e und x^i_e Werte addieren und als capacity Werte eintragen
            for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
                // die capacity ist die summe aus den Werten der ersten Phase und den der i-ten Phase
                //capacity_map[edges[j]] = variables_array[j].get(GRB_DoubleAttr) + varables_array[i * numberEdges + j].get(GRB_DoubleAttr);
                capacity_map[e] = gurobi_variables_map[e][0].get(GRB_DoubleAttr_X) + gurobi_variables_map[e][i].get(GRB_DoubleAttr_X);
            }

            // suche minCut
            lemon::HaoOrlin<lemon::ListGraph, lemon::ListGraph::EdgeMap<double>> hao(two_stage_problem.g, capacity_map);
            hao.init();
            hao.calculateIn();

            // falls der minCut die Bedingung verletzt, der Aufruf speichert direkt auch die bools fuer die Teilmengen in min_cut_result_map
            min_cut_value = hao.minCutMap(min_cut_result_map);
            if(min_cut_value < 1) {
                
                // fuege neues constraint hinzu, damit diese Bedingung in zukunft erfuellt ist
                GRBLinExpr constraint = 0.0;

                // dazu muss ich erstmal alle Kanten ermitteln, die bei diesem minCut die beiden Teilmengen verbinden
                for (lemon::ListGraph::NodeIt n(two_stage_problem.g); n != lemon::INVALID; ++n) {
                    if (min_cut_result_map[n]) {
                        for (lemon::ListGraph::IncEdgeIt e(two_stage_problem.g, n); e != lemon::INVALID; ++e) {
                            // wenn entweder n true und die andere edge false ist oder andersrum
                            if ((min_cut_result_map[two_stage_problem.g.source(e)] && !min_cut_result_map[two_stage_problem.g.target(e)]) || (!min_cut_result_map[two_stage_problem.g.source(e)] && min_cut_result_map[two_stage_problem.g.target(e)])) {

                                constraint += gurobi_variables_map[e][0] + gurobi_variables_map[e][i];

                            }
                        }
                    }
                }

                // constraint jetzt noch hinzufuegen
                model.addConstr(constraint, GRB_GREATER_EQUAL, 1.0);

                // und ich gehe aus derm for-loop raus, in der Annahme, dass allein diese Veraenderung schon was bewirkt und es sich vielleicht nicht lohnt, noch weiter durch alle anderen
                //Szenarien zu schauen
                break;
            } 

        }
        // hier hab ich entweder alle szenarien durch und nicht gebreakt, was bedeutet, dass alle min-cut-values >= 1 sind oder ich bin entweder aus dem for-loop rausgebreakt, also
        // muss ich nochmal checken, ob auch der letzte Wert >= 1 ist, weil wenn nicht, muss ich neu mit der neuen Bed. optimieren

        // falls an diesem Punkt der minCut das Constraint erfuelt, gibt es kein Szenario mehr, wo der minCut gegen das Constraint verstoest und ich bin fertig mit der LP-Loesung
        if(min_cut_value >= 1) {
            break;
        }
        // ansonsten optimiere erneut
    }


    // TIMER
    auto t_end_loop = std::chrono::high_resolution_clock::now();
    loop_zeit = std::chrono::duration_cast<std::chrono::seconds>(t_end_loop - t_start_loop);

    //std::cout << "Loop-Zeit: " << loop_int.count() << "s\n";

    //std::cout << "Anzahl an Iterationen im Loop: " << counter << "\n";
    // ENDE TIMER

    // DEBUGGING::
    // std::cout << "DEBUGGING" << std::endl;
    // auto e = two_stage_problem.g.edgeFromId(0);
    // auto a = model.get(GRB_DoubleAttr_X, gurobi_variables_map[e], two_stage_problem.numberScenarios+1);
    // std::cout<< a[0] << std::endl;
    // std::cout << a[1] << std::endl;
    // std::cout << "Anzahl an Szenarios hier:" << std::endl;
    // ENDE DEBUGGING!!

    double res = model.get(GRB_DoubleAttr_ObjVal);              // res ist der Wert der objective function
    //double res = obj.getValue();
    
    // ich schreibe nun in die uebergebene EdgeMap die Ergebnisse der optimierten LP-Variablen und free die hier allocateten Variablen arrays
    for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {

        // std::copy_n(model.get(GRB_DoubleAttr_X, gurobi_variables_map[e], two_stage_problem.numberScenarios+1), two_stage_problem.lp_results_map[e].size(), two_stage_problem.lp_results_map[e].begin());

        double * lp_edge_solutions = model.get(GRB_DoubleAttr_X, gurobi_variables_map[e], two_stage_problem.numberScenarios+1);
        for (int i=0; i<two_stage_problem.numberScenarios+1; i++) {
            two_stage_problem.lp_results_map[e].push_back(lp_edge_solutions[i]);
        }

        // hier free ich die Variablen arrays pro Kante
        delete[] gurobi_variables_map[e];

    }

    // for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        // delete[] gurobi_variables_map[e];
    // }

    //vielleicht brauche ich auch gar nicht die Variablen zurueckgeben, sondern nur deren Werte

    // return gurobi_variables_map;

    return res;
}


