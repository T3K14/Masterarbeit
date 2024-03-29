#include "two_stage_gurobi.hpp"
#include "two_stage.hpp"
#include <lemon/hao_orlin.h>
#include <algorithm>
#include <iostream>

#include <chrono>


// nimmt ein two_stage_problem und loesst das mit hilfe von Gurobi, mit neuen Constraints
double solve_relaxed_lp(TwoStageProblem & two_stage_problem) {
    
    try {
        
        // Setup: jede Kante bekommt ein Array von Gurobi-Variablen, eine fuer Stage 1 und jeweils eine fuer jedes Szenario in Stage 2
        GRBEnv env = GRBEnv();

        GRBModel model = GRBModel(env);
        model.set(GRB_IntParam_OutputFlag, 0);

        // jeder Kante wird ein Array von GurobiVariablen zugeordnet
        lemon::ListGraph::EdgeMap<GRBVar *> gurobi_variables_map(two_stage_problem.g);

        // das wird die objective function 
        GRBLinExpr obj = 0;

        // baue gleichzeitig die n-1 constraints fuer alle szenarien
        std::vector<GRBLinExpr> c_vec(two_stage_problem.get_number_scenarios()); 

        // ich tue so, als haette ich die edges nicht zwangsweise selbst in einem array sondern nutze den lemon edge iterator
        for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
            
            // jede Kante bekommt array mit Variablen fuer alle Szenarien (+1 fuer die erste Stage)
            gurobi_variables_map[e] = model.addVars(two_stage_problem.numberScenarios + 1, GRB_CONTINUOUS);               // werden unten gefreet

            // und ich kann gleich schon die objective function mit aufbauen
            // stage 1
            obj += two_stage_problem.firstStageWeights[e] * gurobi_variables_map[e][0];

            // adde constraint, dass alle x_e^0 kleiner gleich 1 sein sollen
            model.addConstr(gurobi_variables_map[e][0] <= 1.0);

            // stage 2
            // der index geht bei 0 los, aber von den gurobi_variablen muss ich immer 1 drauf rechnen, weil die nullte variable fuer der erste Stage ist
            for (int i=0; i<two_stage_problem.numberScenarios; i++) {
                obj += two_stage_problem.secondStageProbabilities[i] * two_stage_problem.secondStageWeights[e][i] * gurobi_variables_map[e][i+1];

                // und baue hier noch die n-1 constraints weiter
                c_vec[i] += gurobi_variables_map[e][0] + gurobi_variables_map[e][i+1];

                // adde constraint, dass alle x_e^i kleiner gleich 1 sein sollen
                model.addConstr(gurobi_variables_map[e][i+1] <= 1.0);

                // adde constraint, dass pro Szenario eine einzelne Kante nicht in 2. stage gekauft werden darf, wenn sie in 1. stage schon gekauft wurde
                model.addConstr(gurobi_variables_map[e][0] + gurobi_variables_map[e][i+1] <= 1.0);
            }
        }

        model.setObjective(obj, GRB_MINIMIZE);

        // adde die n-1 constraints
        for (auto & c : c_vec) {
            model.addConstr(c, GRB_EQUAL, two_stage_problem.get_number_nodes() - 1);
        }

        // Ende vom Setup

        unsigned int counter2 = 0;

        // jetzt so lange Cut-Constraints hinzufuegen, bis die Bedingungen immer erfuellt sind 
        while(true) {
            counter2++;

            // wenn der counter zu groess wird, dann muss ich aufpassen, dass kein overvlow stattfindet

            if (counter2 > 18446744073709551613) {       // diese Zahl +2 ist die obere Grenze vom 8 byte unsigned long
                std::cout << "ACHTUNG: integer Overflow findet statt!\n";
            }

            model.optimize();

            double min_cut_value;

            // map der capacities (wird fuer jedes szenario neu beschrieben), brauche ich fuer den HaoOrlin-Algorithmus
            lemon::ListGraph::EdgeMap<double> capacity_map(two_stage_problem.g);

            // map in der angegeben wird, welche Knoten in der einen MinCut-Teilmenge drin sind, output des HaoOrlin-Algorithmus, aus dem ich dann die Kanten bestimmen kann, die die Cut-
            // Teilmengen verbinden
            lemon::ListGraph::NodeMap<bool> min_cut_result_map(two_stage_problem.g);

            // scenario counter, der fuer jedes Szenario hochgezaehlt wird, bei welchem der mincut >= 0.99999999 ist
            int scenario_counter = 0;

            // gehe alle szenarien durch und suche nach mincut, der die Bedingung nicht erfuellt
            for (int i=1; i<two_stage_problem.numberScenarios+1; i++) {     //index geht bei 1 los, weil er nur benutzt wird, um auf die gurobi_variablen zuzugreifen und die haben zum index 0 den Variable fuer die erste stage 

                // die x^_e und x^i_e Werte addieren und als capacity Werte eintragen
                for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
                    // die capacity ist die summe aus den Werten der ersten Phase und den der i-ten Phase
                    capacity_map[e] = gurobi_variables_map[e][0].get(GRB_DoubleAttr_X) + gurobi_variables_map[e][i].get(GRB_DoubleAttr_X);
                }

                // suche minCut
                lemon::HaoOrlin<lemon::ListGraph, lemon::ListGraph::EdgeMap<double>> hao(two_stage_problem.g, capacity_map);
                hao.init();
                hao.calculateIn();

                // falls der minCut die Bedingung verletzt, der Aufruf speichert direkt auch die bools fuer die Teilmengen in min_cut_result_map
                min_cut_value = hao.minCutMap(min_cut_result_map);
                if(min_cut_value < 0.99999999) {
                    
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
                    // break;
                } 
                else {
                    scenario_counter++;
                }
            }
            // hier hab ich entweder alle szenarien durch und nicht gebreakt, was bedeutet, dass alle min-cut-values >= 1 sind oder ich bin entweder aus dem for-loop rausgebreakt, also
            // muss ich nochmal checken, ob auch der letzte Wert >= 1 ist, weil wenn nicht, muss ich neu mit der neuen Bed. optimieren

            // falls an diesem Punkt der minCut das Constraint erfuelt, gibt es kein Szenario mehr, wo der minCut gegen das Constraint verstoest und ich bin fertig mit der LP-Loesung
            if (scenario_counter == two_stage_problem.numberScenarios) {
                break;
            }

            // ansonsten optimiere erneut
        }

        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimum: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
        }
        else {
            std::cout << "No Solution!" << std::endl;
            throw std::logic_error("ROBERTERROR: ES GIBT KEINE GUELTIGE GUROBI LOESUNG!");
        }

        double res = model.get(GRB_DoubleAttr_ObjVal);              // res ist der Wert der objective function
    
        // ich schreibe nun in die uebergebene EdgeMap die Ergebnisse der optimierten LP-Variablen und free die hier allocateten Variablen arrays
        for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {

            double * lp_edge_solutions = model.get(GRB_DoubleAttr_X, gurobi_variables_map[e], two_stage_problem.numberScenarios+1);
            for (int i=0; i<two_stage_problem.numberScenarios+1; i++) {
                two_stage_problem.lp_results_map[e].push_back(lp_edge_solutions[i]);
            }

            // hier free ich die Variablen arrays pro Kante
            delete[] gurobi_variables_map[e];

        }

        return res;
    }
    catch (GRBException e)
    {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
    }
}


/*
// NUR ZUM TESTEN: VERAENDERT SICH DIE PERFORMANCE VOM LP-ALG, WENN DIE EXTRA CONSTRAINTS RAUSFALLEN??
double solve_relaxed_lp(TwoStageProblem & two_stage_problem) {
    
    try {
        
        // Setup: jede Kante bekommt ein Array von Gurobi-Variablen, eine fuer Stage 1 und jeweils eine fuer jedes Szenario in Stage 2
        GRBEnv env = GRBEnv();

        GRBModel model = GRBModel(env);
        model.set(GRB_IntParam_OutputFlag, 0);

        // jeder Kante wird ein Array von GurobiVariablen zugeordnet
        lemon::ListGraph::EdgeMap<GRBVar *> gurobi_variables_map(two_stage_problem.g);

        // das wird die objective function 
        GRBLinExpr obj = 0;

        // baue gleichzeitig die n-1 constraints fuer alle szenarien
        std::vector<GRBLinExpr> c_vec(two_stage_problem.get_number_scenarios()); 

        // ich tue so, als haette ich die edges nicht zwangsweise selbst in einem array sondern nutze den lemon edge iterator
        for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
            
            // jede Kante bekommt array mit Variablen fuer alle Szenarien (+1 fuer die erste Stage)
            gurobi_variables_map[e] = model.addVars(two_stage_problem.numberScenarios + 1, GRB_CONTINUOUS);               // werden unten gefreet

            // und ich kann gleich schon die objective function mit aufbauen
            // stage 1
            obj += two_stage_problem.firstStageWeights[e] * gurobi_variables_map[e][0];

            // adde constraint, dass alle x_e^0 kleiner gleich 1 sein sollen
            model.addConstr(gurobi_variables_map[e][0] <= 1.0);

            // stage 2
            // der index geht bei 0 los, aber von den gurobi_variablen muss ich immer 1 drauf rechnen, weil die nullte variable fuer der erste Stage ist
            for (int i=0; i<two_stage_problem.numberScenarios; i++) {
                obj += two_stage_problem.secondStageProbabilities[i] * two_stage_problem.secondStageWeights[e][i] * gurobi_variables_map[e][i+1];

                // und baue hier noch die n-1 constraints weiter
                c_vec[i] += gurobi_variables_map[e][0] + gurobi_variables_map[e][i+1];

                // adde constraint, dass alle x_e^i kleiner gleich 1 sein sollen
                model.addConstr(gurobi_variables_map[e][i+1] <= 1.0);

                // RAUSGENOMMEN:
                // adde constraint, dass pro Szenario eine einzelne Kante nicht in 2. stage gekauft werden darf, wenn sie in 1. stage schon gekauft wurde
                // model.addConstr(gurobi_variables_map[e][0] + gurobi_variables_map[e][i+1] <= 1.0);
            }
        }

        model.setObjective(obj, GRB_MINIMIZE);

        // RAUSGENOMMEN:
        // adde die n-1 constraints
        // for (auto & c : c_vec) {
        //     model.addConstr(c, GRB_EQUAL, two_stage_problem.get_number_nodes() - 1);
        // }

        // Ende vom Setup

        unsigned int counter2 = 0;

        // jetzt so lange Cut-Constraints hinzufuegen, bis die Bedingungen immer erfuellt sind 
        while(true) {
            counter2++;

            // wenn der counter zu groess wird, dann muss ich aufpassen, dass kein overvlow stattfindet

            if (counter2 > 18446744073709551613) {       // diese Zahl +2 ist die obere Grenze vom 8 byte unsigned long
                std::cout << "ACHTUNG: integer Overflow findet statt!\n";
            }

            model.optimize();

            double min_cut_value;

            // map der capacities (wird fuer jedes szenario neu beschrieben), brauche ich fuer den HaoOrlin-Algorithmus
            lemon::ListGraph::EdgeMap<double> capacity_map(two_stage_problem.g);

            // map in der angegeben wird, welche Knoten in der einen MinCut-Teilmenge drin sind, output des HaoOrlin-Algorithmus, aus dem ich dann die Kanten bestimmen kann, die die Cut-
            // Teilmengen verbinden
            lemon::ListGraph::NodeMap<bool> min_cut_result_map(two_stage_problem.g);

            // scenario counter, der fuer jedes Szenario hochgezaehlt wird, bei welchem der mincut >= 0.99999999 ist
            int scenario_counter = 0;

            // gehe alle szenarien durch und suche nach mincut, der die Bedingung nicht erfuellt
            for (int i=1; i<two_stage_problem.numberScenarios+1; i++) {     //index geht bei 1 los, weil er nur benutzt wird, um auf die gurobi_variablen zuzugreifen und die haben zum index 0 den Variable fuer die erste stage 

                // die x^_e und x^i_e Werte addieren und als capacity Werte eintragen
                for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
                    // die capacity ist die summe aus den Werten der ersten Phase und den der i-ten Phase
                    capacity_map[e] = gurobi_variables_map[e][0].get(GRB_DoubleAttr_X) + gurobi_variables_map[e][i].get(GRB_DoubleAttr_X);
                }

                // suche minCut
                lemon::HaoOrlin<lemon::ListGraph, lemon::ListGraph::EdgeMap<double>> hao(two_stage_problem.g, capacity_map);
                hao.init();
                hao.calculateIn();

                // falls der minCut die Bedingung verletzt, der Aufruf speichert direkt auch die bools fuer die Teilmengen in min_cut_result_map
                min_cut_value = hao.minCutMap(min_cut_result_map);
                if(min_cut_value < 0.99999999) {
                    
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
                    // break;
                } 
                else {
                    scenario_counter++;
                }
            }
            // hier hab ich entweder alle szenarien durch und nicht gebreakt, was bedeutet, dass alle min-cut-values >= 1 sind oder ich bin entweder aus dem for-loop rausgebreakt, also
            // muss ich nochmal checken, ob auch der letzte Wert >= 1 ist, weil wenn nicht, muss ich neu mit der neuen Bed. optimieren

            // falls an diesem Punkt der minCut das Constraint erfuelt, gibt es kein Szenario mehr, wo der minCut gegen das Constraint verstoest und ich bin fertig mit der LP-Loesung
            if (scenario_counter == two_stage_problem.numberScenarios) {
                break;
            }

            // ansonsten optimiere erneut
        }

        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimum: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
        }
        else {
            std::cout << "No Solution!" << std::endl;
            throw std::logic_error("ROBERTERROR: ES GIBT KEINE GUELTIGE GUROBI LOESUNG!");
        }

        double res = model.get(GRB_DoubleAttr_ObjVal);              // res ist der Wert der objective function
    
        // ich schreibe nun in die uebergebene EdgeMap die Ergebnisse der optimierten LP-Variablen und free die hier allocateten Variablen arrays
        for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {

            double * lp_edge_solutions = model.get(GRB_DoubleAttr_X, gurobi_variables_map[e], two_stage_problem.numberScenarios+1);
            for (int i=0; i<two_stage_problem.numberScenarios+1; i++) {
                two_stage_problem.lp_results_map[e].push_back(lp_edge_solutions[i]);
            }

            // hier free ich die Variablen arrays pro Kante
            delete[] gurobi_variables_map[e];

        }

        return res;
    }
    catch (GRBException e)
    {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
    }
}
*/


// nimmt ein two_stage_problem und loesst das mit hilfe von Gurobi und trackt die Zeit, modifiziert mit neuen constraints
double solve_relaxed_lp(TwoStageProblem & two_stage_problem, unsigned long & counter, double & setup_zeit_ms, double & loop_zeit_s, std::vector<double> & opt_times_ms, std::vector<double> & add_constr_times_s) { //, lemon::ListGraph::EdgeMap<std::vector<double>> & two_stage_problem.lp_results_map) {

    auto t_start_setup = std::chrono::high_resolution_clock::now();

    try {

        // DEBUG
        // for (auto i : two_stage_problem.edges) {
        //     std::cout << two_stage_problem.firstStageWeights[i] << "\n";
        // }

        // std:: cout << "Das waren die first stage weights" << std::endl;

        // std::cout << "pkt 1 in Funktion\n";
        // auto eddy = two_stage_problem.g.edgeFromId(0);

        // std::cout << "prob:" << two_stage_problem.secondStageProbabilities[0] << "\n";
        // std::cout << "first:" << two_stage_problem.firstStageWeights[eddy] << "\n";
        // std::cout << "second:" << two_stage_problem.secondStageWeights[eddy][0] << "\n";
        //ENDE DEBUG /

        // Setup: jede Kante bekommt ein Array von Gurobi-Variablen, eine fuer Stage 1 und jeweils eine fuer jedes Szenario in Stage 2
        GRBEnv env = GRBEnv();

        GRBModel model = GRBModel(env);
        model.set(GRB_IntParam_OutputFlag, 0);

        // jeder Kante wird ein Array von GurobiVariablen zugeordnet
        lemon::ListGraph::EdgeMap<GRBVar *> gurobi_variables_map(two_stage_problem.g);

        // das wird die objective function 
        GRBLinExpr obj = 0;

        // baue gleichzeitig die n-1 constraints fuer alle szenarien
        std::vector<GRBLinExpr> c_vec(two_stage_problem.get_number_scenarios()); 

        // ich tue so, als haette ich die edges nicht zwangsweise selbst in einem array sondern nutze den lemon edge iterator
        for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
            
            // jede Kante bekommt array mit Variablen fuer alle Szenarien (+1 fuer die erste Stage)
            gurobi_variables_map[e] = model.addVars(two_stage_problem.numberScenarios + 1, GRB_CONTINUOUS);               // werden unten gefreet

            // und ich kann gleich schon die objective function mit aufbauen
            // stage 1
            obj += two_stage_problem.firstStageWeights[e] * gurobi_variables_map[e][0];

            // adde constraint, dass alle x_e^0 kleiner gleich 1 sein sollen
            model.addConstr(gurobi_variables_map[e][0] <= 1.0);

            // stage 2
            // der index geht bei 0 los, aber von den gurobi_variablen muss ich immer 1 drauf rechnen, weil die nullte variable fuer der erste Stage ist
            for (int i=0; i<two_stage_problem.numberScenarios; i++) {
                obj += two_stage_problem.secondStageProbabilities[i] * two_stage_problem.secondStageWeights[e][i] * gurobi_variables_map[e][i+1];

                // und baue hier noch die n-1 constraints weiter
                c_vec[i] += gurobi_variables_map[e][0] + gurobi_variables_map[e][i+1];

                // adde constraint, dass alle x_e^i kleiner gleich 1 sein sollen
                model.addConstr(gurobi_variables_map[e][i+1] <= 1.0);

                // adde constraint, dass pro Szenario eine einzelne Kante nicht in 2. stage gekauft werden darf, wenn sie in 1. stage schon gekauft wurde
                model.addConstr(gurobi_variables_map[e][0] + gurobi_variables_map[e][i+1] <= 1.0);
            }
        }

        // DEBUG
        // std::cout << "Pkt 2 in Funktion\n";
        // std::cout << obj.size() << std::endl; 
        // std::cout << obj.getCoeff(0) << ", und " << obj.getCoeff(1) << std::endl;
        //ENDE DEBUG //

        model.setObjective(obj, GRB_MINIMIZE);

        // adde die n-1 constraints
        for (auto & c : c_vec) {
            model.addConstr(c, GRB_EQUAL, two_stage_problem.get_number_nodes() - 1);
        }

        // Ende vom Setup

        // TIMER bis hierhin und ab hier timen und Iterationen zaehlen

        auto t_end_setup = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fs_ms = t_end_setup - t_start_setup;
        setup_zeit_ms = fs_ms.count();

        //std::cout << "Setup-Zeit: " << setup_int.count() << "s\n";

        auto t_start_loop = std::chrono::high_resolution_clock::now();

        // unsigned long counter = 0;

        // vector, in den ich die Zeiten reinpacke, wie lange es gedauert hat, einmal zu optimieren.
        //std::vector<std::chrono::milliseconds> opti_times;

        // ENDE TIMER

        unsigned int counter2 = 0;

        // jetzt so lange Cut-Constraints hinzufuegen, bis die Bedingungen immer erfuellt sind 
        while(true) {
            counter2++;
            std::cout << "counter2: " << counter2 << std::endl;

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
            std::chrono::duration<double, std::milli> fp_ms = t_end_opt - t_start_opt;
            opt_times_ms.push_back(fp_ms.count());

            // DEBUG
            // std::cout << model.get(GRB_DoubleAttr_Runtime) << std::endl;

            // // Debug, speichere die Ergebnisse der einzel gurobi variablen, um zu schauen, ob nach 1. gurobi Schritt die constraints erfuellt sind
            // std::ofstream out_gur;
            // out_gur.open("/gss/work/xees8992/OutGur.txt", std::ios_base::out);
            
            // for (auto eddie : two_stage_problem.edges) {
            //     // out_gur << "Edge: " << two_stage_problem.g.id(eddie) << ": " << gurobi_variables_map[eddie][0].get(GRB_DoubleAttr_X);
            //     out_gur << "Edge" << two_stage_problem.g.id(eddie) << ": " << gurobi_variables_map[eddie][0].get(GRB_DoubleAttr_X);

            //     for (int ii=0; ii<two_stage_problem.numberScenarios; ii++) {
            //         out_gur << ", " << gurobi_variables_map[eddie][ii+1].get(GRB_DoubleAttr_X);
            //     }
            //     out_gur << "\n";
            // }
            // out_gur.close();
            // // ENDE Debug,

            // // bruteforce Beenden
            // throw std::logic_error("ROBERTERROR: Stop!");

            // DEBUG
            // std::cout << "\n\n Hier nach einem optimierungsvorgang\n";
            // // gebe wert der ersten variablen aus und der objective function
            // std::cout <<  "Objective:" << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
            // std::cout << gurobi_variables_map[two_stage_problem.g.edgeFromId(0)][0].get(GRB_DoubleAttr_X) << std::endl;
            // std::cout << gurobi_variables_map[two_stage_problem.g.edgeFromId(0)][1].get(GRB_DoubleAttr_X) << std::endl;
            // ENDE DEBUG

            auto t_start_rest = std::chrono::high_resolution_clock::now();


            double min_cut_value;

            // map der capacities (wird fuer jedes szenario neu beschrieben), brauche ich fuer den HaoOrlin-Algorithmus
            lemon::ListGraph::EdgeMap<double> capacity_map(two_stage_problem.g);

            // map in der angegeben wird, welche Knoten in der einen MinCut-Teilmenge drin sind, output des HaoOrlin-Algorithmus, aus dem ich dann die Kanten bestimmen kann, die die Cut-
            // Teilmengen verbinden
            lemon::ListGraph::NodeMap<bool> min_cut_result_map(two_stage_problem.g);

            // scenario counter, der fuer jedes Szenario hochgezaehlt wird, bei welchem der mincut >= 0.99999999 ist
            int scenario_counter = 0;

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
                if(min_cut_value < 0.99999999) {
                    
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
                    // break;
                } 
                else {
                    scenario_counter++;
                }
            }
            // hier hab ich entweder alle szenarien durch und nicht gebreakt, was bedeutet, dass alle min-cut-values >= 1 sind oder ich bin entweder aus dem for-loop rausgebreakt, also
            // muss ich nochmal checken, ob auch der letzte Wert >= 1 ist, weil wenn nicht, muss ich neu mit der neuen Bed. optimieren

            // std::cout << "min_cut_value: " << min_cut_value << std::endl;

            auto t_end_rest = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> fres_s = t_end_rest - t_start_rest;
            add_constr_times_s.push_back(fres_s.count());

            // falls an diesem Punkt der minCut das Constraint erfuelt, gibt es kein Szenario mehr, wo der minCut gegen das Constraint verstoest und ich bin fertig mit der LP-Loesung
            if (scenario_counter == two_stage_problem.numberScenarios) {
                break;
            }

            // ansonsten optimiere erneut
        }

        // TIMER
        auto t_end_loop = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> fl_s = t_end_loop - t_start_loop;
        loop_zeit_s = fl_s.count();

        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimum: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
        }
        else {
            std::cout << "No Solution!" << std::endl;
            throw std::logic_error("ROBERTERROR: ES GIBT KEINE GUELTIGE GUROBI LOESUNG!");
        }
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
    catch (GRBException e)
    {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
    }
    // catch (const std::exception& ex)
    // {
    //     std::cout << "Error occurred: " << ex.what() << std::endl;
    // }
    // catch(...) {
    //     std::cout << "Anderer Error" << std::endl;
    // }
}

/*
// nimmt ein two_stage_problem und loesst das mit hilfe von Gurobi und trackt die Zeit, OHNE NEUE CONSTRAINTS
double solve_relaxed_lp(TwoStageProblem & two_stage_problem, unsigned long & counter, double & setup_zeit_ms, double & loop_zeit_s, std::vector<double> & opt_times_ms, std::vector<double> & add_constr_times_s) { //, lemon::ListGraph::EdgeMap<std::vector<double>> & two_stage_problem.lp_results_map) {

    auto t_start_setup = std::chrono::high_resolution_clock::now();

    try {

        // DEBUG
        // for (auto i : two_stage_problem.edges) {
        //     std::cout << two_stage_problem.firstStageWeights[i] << "\n";
        // }

        // std:: cout << "Das waren die first stage weights" << std::endl;

        // std::cout << "pkt 1 in Funktion\n";
        // auto eddy = two_stage_problem.g.edgeFromId(0);

        // std::cout << "prob:" << two_stage_problem.secondStageProbabilities[0] << "\n";
        // std::cout << "first:" << two_stage_problem.firstStageWeights[eddy] << "\n";
        // std::cout << "second:" << two_stage_problem.secondStageWeights[eddy][0] << "\n";
        ENDE DEBUG //

        // Setup: jede Kante bekommt ein Array von Gurobi-Variablen, eine fuer Stage 1 und jeweils eine fuer jedes Szenario in Stage 2
        GRBEnv env = GRBEnv();

        GRBModel model = GRBModel(env);
        model.set(GRB_IntParam_OutputFlag, 0);

        // jeder Kante wird ein Array von GurobiVariablen zugeordnet
        lemon::ListGraph::EdgeMap<GRBVar *> gurobi_variables_map(two_stage_problem.g);

        // das wird die objective function 
        GRBLinExpr obj = 0;

        // baue gleichzeitig die n-1 constraints fuer alle szenarien
        // std::vector<GRBLinExpr> c_vec(two_stage_problem.get_number_scenarios()); 

        // ich tue so, als haette ich die edges nicht zwangsweise selbst in einem array sondern nutze den lemon edge iterator
        for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
            
            // jede Kante bekommt array mit Variablen fuer alle Szenarien (+1 fuer die erste Stage)
            gurobi_variables_map[e] = model.addVars(two_stage_problem.numberScenarios + 1, GRB_CONTINUOUS);               // werden unten gefreet

            // und ich kann gleich schon die objective function mit aufbauen
            // stage 1
            obj += two_stage_problem.firstStageWeights[e] * gurobi_variables_map[e][0];

            // adde constraint, dass alle x_e^0 kleiner gleich 1 sein sollen
            model.addConstr(gurobi_variables_map[e][0] <= 1.0);

            // stage 2
            // der index geht bei 0 los, aber von den gurobi_variablen muss ich immer 1 drauf rechnen, weil die nullte variable fuer der erste Stage ist
            for (int i=0; i<two_stage_problem.numberScenarios; i++) {
                obj += two_stage_problem.secondStageProbabilities[i] * two_stage_problem.secondStageWeights[e][i] * gurobi_variables_map[e][i+1];

                // und baue hier noch die n-1 constraints weiter
                // c_vec[i] += gurobi_variables_map[e][0] + gurobi_variables_map[e][i+1];

                // adde constraint, dass alle x_e^i kleiner gleich 1 sein sollen
                model.addConstr(gurobi_variables_map[e][i+1] <= 1.0);

                // adde constraint, dass pro Szenario eine einzelne Kante nicht in 2. stage gekauft werden darf, wenn sie in 1. stage schon gekauft wurde
                // model.addConstr(gurobi_variables_map[e][0] + gurobi_variables_map[e][i+1] <= 1.0);
            }
        }

        // DEBUG
        // std::cout << "Pkt 2 in Funktion\n";
        // std::cout << obj.size() << std::endl; 
        // std::cout << obj.getCoeff(0) << ", und " << obj.getCoeff(1) << std::endl;
        ENDE DEBUG //

        model.setObjective(obj, GRB_MINIMIZE);

        // adde die n-1 constraints
        // for (auto & c : c_vec) {
        //     model.addConstr(c, GRB_EQUAL, two_stage_problem.get_number_nodes() - 1);
        // }

        // Ende vom Setup

        // TIMER bis hierhin und ab hier timen und Iterationen zaehlen

        auto t_end_setup = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fs_ms = t_end_setup - t_start_setup;
        setup_zeit_ms = fs_ms.count();

        //std::cout << "Setup-Zeit: " << setup_int.count() << "s\n";

        auto t_start_loop = std::chrono::high_resolution_clock::now();

        // unsigned long counter = 0;

        // vector, in den ich die Zeiten reinpacke, wie lange es gedauert hat, einmal zu optimieren.
        //std::vector<std::chrono::milliseconds> opti_times;

        // ENDE TIMER

        unsigned int counter2 = 0;

        // jetzt so lange Cut-Constraints hinzufuegen, bis die Bedingungen immer erfuellt sind 
        while(true) {
            counter2++;
            std::cout << "counter2: " << counter2 << std::endl;

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
            std::chrono::duration<double, std::milli> fp_ms = t_end_opt - t_start_opt;
            opt_times_ms.push_back(fp_ms.count());

            // DEBUG
            // std::cout << model.get(GRB_DoubleAttr_Runtime) << std::endl;

            // // Debug, speichere die Ergebnisse der einzel gurobi variablen, um zu schauen, ob nach 1. gurobi Schritt die constraints erfuellt sind
            // std::ofstream out_gur;
            // out_gur.open("/gss/work/xees8992/OutGur.txt", std::ios_base::out);
            
            // for (auto eddie : two_stage_problem.edges) {
            //     // out_gur << "Edge: " << two_stage_problem.g.id(eddie) << ": " << gurobi_variables_map[eddie][0].get(GRB_DoubleAttr_X);
            //     out_gur << "Edge" << two_stage_problem.g.id(eddie) << ": " << gurobi_variables_map[eddie][0].get(GRB_DoubleAttr_X);

            //     for (int ii=0; ii<two_stage_problem.numberScenarios; ii++) {
            //         out_gur << ", " << gurobi_variables_map[eddie][ii+1].get(GRB_DoubleAttr_X);
            //     }
            //     out_gur << "\n";
            // }
            // out_gur.close();
            // // ENDE Debug,

            // // bruteforce Beenden
            // throw std::logic_error("ROBERTERROR: Stop!");

            // DEBUG
            // std::cout << "\n\n Hier nach einem optimierungsvorgang\n";
            // // gebe wert der ersten variablen aus und der objective function
            // std::cout <<  "Objective:" << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
            // std::cout << gurobi_variables_map[two_stage_problem.g.edgeFromId(0)][0].get(GRB_DoubleAttr_X) << std::endl;
            // std::cout << gurobi_variables_map[two_stage_problem.g.edgeFromId(0)][1].get(GRB_DoubleAttr_X) << std::endl;
            // ENDE DEBUG

            auto t_start_rest = std::chrono::high_resolution_clock::now();


            double min_cut_value;

            // map der capacities (wird fuer jedes szenario neu beschrieben), brauche ich fuer den HaoOrlin-Algorithmus
            lemon::ListGraph::EdgeMap<double> capacity_map(two_stage_problem.g);

            // map in der angegeben wird, welche Knoten in der einen MinCut-Teilmenge drin sind, output des HaoOrlin-Algorithmus, aus dem ich dann die Kanten bestimmen kann, die die Cut-
            // Teilmengen verbinden
            lemon::ListGraph::NodeMap<bool> min_cut_result_map(two_stage_problem.g);

            // scenario counter, der fuer jedes Szenario hochgezaehlt wird, bei welchem der mincut >= 0.99999999 ist
            int scenario_counter = 0;

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
                if(min_cut_value < 0.99999999) {
                    
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
                    // break;
                } 
                else {
                    scenario_counter++;
                }
            }
            // hier hab ich entweder alle szenarien durch und nicht gebreakt, was bedeutet, dass alle min-cut-values >= 1 sind oder ich bin entweder aus dem for-loop rausgebreakt, also
            // muss ich nochmal checken, ob auch der letzte Wert >= 1 ist, weil wenn nicht, muss ich neu mit der neuen Bed. optimieren

            // std::cout << "min_cut_value: " << min_cut_value << std::endl;

            auto t_end_rest = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> fres_s = t_end_rest - t_start_rest;
            add_constr_times_s.push_back(fres_s.count());

            // falls an diesem Punkt der minCut das Constraint erfuelt, gibt es kein Szenario mehr, wo der minCut gegen das Constraint verstoest und ich bin fertig mit der LP-Loesung
            if (scenario_counter == two_stage_problem.numberScenarios) {
                break;
            }

            // ansonsten optimiere erneut
        }

        // TIMER
        auto t_end_loop = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> fl_s = t_end_loop - t_start_loop;
        loop_zeit_s = fl_s.count();

        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimum: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
        }
        else {
            std::cout << "No Solution!" << std::endl;
            throw std::logic_error("ROBERTERROR: ES GIBT KEINE GUELTIGE GUROBI LOESUNG!");
        }
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
    catch (GRBException e)
    {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
    }
    // catch (const std::exception& ex)
    // {
    //     std::cout << "Error occurred: " << ex.what() << std::endl;
    // }
    // catch(...) {
    //     std::cout << "Anderer Error" << std::endl;
    // }
}
*/