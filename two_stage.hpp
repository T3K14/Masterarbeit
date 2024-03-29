/*
Copyright (C) 2003-2012 Egerváry Jenő Kombinatorikus Optimalizálási
Kutatócsoport (Egerváry Research Group on Combinatorial Optimization,
EGRES).
*/


#pragma once

#include <lemon/list_graph.h>
#include <random>
#include <memory>
// #include <lemon/maps.h>
#include <functional> // for reference_wrapper
#include <string>

#include <chrono>

#include "boost/filesystem.hpp"

using boost_path = boost::filesystem::path;

// struct RNG{
    // std::mt19937 eng;
// };

extern std::mt19937 rng;

// forward declaration
class NewEdgeCostCreator;
class ScenarioCreator;
// class NewEdgeCostCreator::override_costs(TwoStageProblem & tsp, std::vector<double> & first_stage_costs, std::vector<std::vector<double>> & second_stage_costs);

class EdgeCostCreator {

    private:
        // RNG & rng;
        // std::uniform_real_distribution<double> uniformDist;


    public:
        ~EdgeCostCreator() = default;
        // EdgeCostCreator(RNG & randGen);
        // lemon::ListGraph::EdgeMap<double> createUniformCosts(const lemon::ListGraph & g, double a, double b);
        std::unique_ptr<lemon::ListGraph::EdgeMap<double>> createUniformCosts(const lemon::ListGraph & g, double a, double b, std::mt19937 & rng);


        // FALLS ICH MAL PARALLELISIEREN MUSS UND DABEI ANDEREN RNG ALS MT19937 NUTZEN WILL/MUSS
        // template<typename RNG>
        // void doRandomStuff(float x, RNG && rng);            // perfect forwarding, universial reference (Scott Meyers)

};

// calculates the MST and returns the total weight of the MST
template<typename T>        // T is the edge cost type
T twoStageSetting(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<T> & firstStageCosts, const lemon::ListGraph::EdgeMap<T> & secondStageCosts, bool save=false);

// calculates the edgeset that minimizes the expected total costs of the MST for a given second stage edgeset-distribution, by trying out every possible combination
// template<typename T>        // T is the edge cost type
// void bruteForceEnumeration(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<T> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::reference_wrapper<lemon::ListGraph::EdgeMap<T>>> & scenarioSecondStageCosts);
double bruteForceEnumeration(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> & scenarioSecondStageCosts);

// works with EdgeMap<vector<double>> instead of vector<unique_ptr<EdgeMap>>
double bruteForceEnumeration(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const lemon::ListGraph::EdgeMap<std::vector<double>> & scenarioSecondStageCostsEM);

double check(const lemon::ListGraph & g, const std::vector<int> & c, const std::vector<lemon::ListGraph::Edge> & edges, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> & scenarioSecondStageCosts, lemon::ListGraph::EdgeMap<bool> & output, double & currentBest, size_t & optCounter, lemon::ListGraph::EdgeMap<bool> & currentFirstStageSelection);

// works with EdgeMap<vector<double>> instead of vector<unique_ptr<EdgeMap>>
double check(const lemon::ListGraph & g, const std::vector<int> & c, const std::vector<lemon::ListGraph::Edge> & edges, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<lemon::ListGraph::EdgeMap<double>> & scenarioSecondStageCosts, lemon::ListGraph::EdgeMap<bool> & output, double & currentBest, size_t & optCounter, lemon::ListGraph::EdgeMap<bool> & currentFirstStageSelection);

double fourb(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const lemon::ListGraph::EdgeMap<std::vector<double>> & scenarioSecondStageCostsEM);


template<typename T>
class SecondStageMap {

    // const lemon::ListGraph::EdgeMap<double> & secondStageCosts;
    const T & ursprungs_map;        // die Map (entweder EdgeMap<double> oder OneScenarioMap)
    const std::vector<lemon::ListGraph::Edge> & edges;
    const std::vector<int> & c;

public:

    typedef double Value;
    typedef lemon::ListGraph::Edge Key;

    Value operator[](Key e) const;
  
    SecondStageMap(const T & s, const std::vector<lemon::ListGraph::Edge> & e, const std::vector<int> & _c);

}; 

// template definition, das zweite typename ist dafuer da, dass der compiler beim parsen versteht, dass SecondStageMap<T>::Value ein typ ist
template<typename T>
typename SecondStageMap<T>::Value SecondStageMap<T>::operator[](SecondStageMap<T>::Key e) const {
    // return orig_len[e]-(pot[g.target(e)]-pot[g.source(e)]);
    
    // HABE DAS HIER AUCH GEAENDERT!!! So gehe ich ueber alle Indizes in c und schaue, ob die Kanten, die dazu gehoeren der uebergebenen entsprechen
    // for (int i=0; i<c.size(); i++) {
    for (int i: c) {
        if (e == edges[i]) {
            return 0;
        }
    }
    return ursprungs_map[e];
}

// die Klasse speichert alles nur als Referenzen, da wird also nichts kopiert   
template<typename T>
SecondStageMap<T>::SecondStageMap(const T & s, const std::vector<lemon::ListGraph::Edge> & e, const std::vector<int> & _c)
    : ursprungs_map(s), edges(e), c(_c) {
    }


// eine Klasse, die mir fuer eine secondstage EdgeMap<vector>-map die Kosten nur fuer ein bestimmtes szenario wiedergibt
class OneScenarioMap {

    // ich nutze hier nur eine Referenz in der Hoffnung, dass es das effizienter macht, und muss daher dafuer sorgen, dass keine Instanz dieser Klasse laenger im scope ist
    // als die Objekte, die in ihr referenziert werden
    const lemon::ListGraph::EdgeMap<std::vector<double>> & secondStageWeights;
    const unsigned int i;

public:
    typedef double Value;
    typedef lemon::ListGraph::Edge Key;

    Value operator[](Key e) const;
    OneScenarioMap(const lemon::ListGraph::EdgeMap<std::vector<double>> & s, const unsigned int _i);
};

// eine Klasse, die mir fuer eine secondstage EdgeMap<vector>-map die Kosten nur fuer ein bestimmtes szenario wiedergibt
// und ausserdem beruecksichtig, wenn eine Kante bereits in der ersten Stage gekauft wurde
// das ist quasi eine kombination aus OneScenarioMap und SecondStageMap, die anderen beiden brauche ich aber noch, weil ich in
// der check-methode nicht ueber die edges loope und mit einer bool map abfrage, sondern mit dem c-vector arbeite
class OneScenarioSecondStageMap {

    // ich nutze hier nur eine Referenz in der Hoffnung, dass es das effizienter macht, und muss daher dafuer sorgen, dass keine Instanz dieser Klasse laenger im scope ist
    // als die Objekte, die in ihr referenziert werden
    const lemon::ListGraph::EdgeMap<std::vector<double>> & secondStageWeights;
    const unsigned int i;
    const lemon::ListGraph::EdgeMap<bool> & boolMap;

public:
    typedef double Value;
    typedef lemon::ListGraph::Edge Key;

    Value operator[](Key e) const;
    OneScenarioSecondStageMap(const lemon::ListGraph::EdgeMap<std::vector<double>> & s, const unsigned int _i, const lemon::ListGraph::EdgeMap<bool> & _boolMap);
};

// const std::vector<lemon::ListGraph::EdgeMap<T>> & scenarioSecondStageCosts
// const std::vector<std::reference_wrapper<lemon::ListGraph::EdgeMap<T>>> & scenarioSecondStageCosts

// von der Klasse koennen spaeter subclasses erben, bei denen klar ist, wie der Graph und die Maps dazu intern erstellt werden sollen
class TwoStageProblem {

    virtual void initialise_graph();

// sollte spaeter lieber protected sein
public:
    // kann entweder die zahl an generierten scenarios sein oder die zahl an versuchen, die man beim samplen haben moechte, auch wenn dabei potentiell weniger scenarios
    // am Ende rauskommen
    unsigned int numberScenarios;

    lemon::ListGraph g;
    std::vector<lemon::ListGraph::Node> nodes;
    std::vector<lemon::ListGraph::Edge> edges;

    lemon::ListGraph::EdgeMap<double> firstStageWeights;

    std::vector<double> secondStageProbabilities;
    lemon::ListGraph::EdgeMap<std::vector<double>> secondStageWeights;


    // hier sollen die double Ergebnisse des LP-Algorithmus zwischengespeichert werden
    lemon::ListGraph::EdgeMap<std::vector<double>> lp_results_map;

    // in diesen Maps wird gespeichert, welche Kanten von den jeweiligen Algorithmen fuer die erste Phase eine Kaufempfehlung erhalten

    lemon::ListGraph::EdgeMap<bool> approx_first_stage_map;
    lemon::ListGraph::EdgeMap<bool> bruteforce_first_stage_map;
    lemon::ListGraph::EdgeMap<bool> optimum_first_stage_map;
    lemon::ListGraph::EdgeMap<bool> greedy_first_stage_map;

public:


    // constructor (ist da, damit ich die ganzen member bekomme?)
    TwoStageProblem(const std::vector<double> & second_stage_probabilites);
    
    // den default constructor ueberlade ich so, dass er den ersten constructor mit vec<double> & aufruft, mit einem leeren Vektor
    TwoStageProblem();    
    
    
    
    virtual ~TwoStageProblem() = default;

    // formuliert das relaxed LP Problem, loesst es und speichert die Ergebnisse in 'result_optimized_values_map'
    //friend void solve_relaxed_lp(lemon::ListGraph::EdgeMap<std::vector<double>> & result_optimized_values_map);
    friend double solve_relaxed_lp(TwoStageProblem & two_stage_problem); //, lemon::ListGraph::EdgeMap<std::vector<double>> & result_optimized_values_map);

    // friend void NewEdgeCostCreator::override_costs(TwoStageProblem & tsp, std::vector<double> & first_stage_costs, std::vector<std::vector<double>> & second_stage_costs);
    friend NewEdgeCostCreator;
    friend ScenarioCreator;

    // nur zum zeit testen:
    friend double solve_relaxed_lp(TwoStageProblem & two_stage_problem, unsigned long & counter, double & setup_zeit_ms, double & loop_zeit_s, std::vector<double> & opt_times_ms, std::vector<double> & add_constr_times_s);

    // gibt mir den zu erwartenden 2. stage Kostenwert einer Kante zurueck
    double get_second_stage_ev(const lemon::ListGraph::Edge & e) const; 

    //4b
    double do4b() const;

    // nimmt die gurobi-lp loesung und ermittelt daraus die approximierte Loesung, die in der EdgeMap final_sirst_stage_map die Vorschlaege fuer 
    // in der ersten Stage zu kaufende Kanten speichert
    // void approximate(lemon::ListGraph::EdgeMap<bool> & final_first_stage_map, std::mt19937 & rng);        // WOHER KOMMT DER ENG?????????????
    void approximate(std::mt19937 & rng);        // der rng kommt aus utiities.cpp
    void approximate_old(std::mt19937 & rng);
    // --- BRUTEFORCEFUNKTIONEN
    // berechnet durch durchprobieren aller (sinnvollen) Moeglichkeiten die optimale Loesung fuer das gegebene Problem und speichert die Kanten, die in Phase 1 gekauft werden in 'bruteforce_first_stage_map'
    double bruteforce();
    double bruteforce_new(bool tracking=false, const boost_path & tracking_path=boost_path());
    double optimum(bool tracking=false, const boost_path & tracking_path=boost_path());

    void greedy();

    unsigned int get_number_nodes();
    unsigned int get_number_edges();
    unsigned int get_number_scenarios();

private:
    // schaut sich fuer eine Edgeauswahl an, was dabei herauskommen wuerde und vergleicht das mit dem bisherigen Optimum und ersetzt es, falls das besser ist
    double check(const std::vector<int> & c, double & current_best, lemon::ListGraph::EdgeMap<bool> & output, unsigned int & opt_counter);
    bool check_new(const std::vector<int> & c, double & current_best, lemon::ListGraph::EdgeMap<bool> & output, unsigned int & opt_counter, lemon::ListGraph::EdgeMap<bool> & opt_result_map, bool tracking, const boost_path & tracking_path);

public:

    void save_lp_result_map(const boost_path & output_path);
    // einzige Fkt zum Abspeichern von Ergebnis bool EdgeMaps
    void save_result_map(const lemon::ListGraph::EdgeMap<bool> & bool_map, const boost_path & output_path);

    // --- Funktionen zum abspeichern von den Edgemaps, SIND VERALTET UND NICHT MEHR IM GEBRAUCH
    // void save_approx_result_map(std::string output_name, bool on_cluster=true, bool work=true);
    // void save_bruteforce_first_stage_map(std::string output_name, bool on_cluster=true, bool work=true);
    // void save_greedy_first_stage_map(std::string output_path, bool on_cluster=true);

    // Funktion, die mir fuer eine gegebene bool-map die zu erwartenden Kosten ausrechnet
    double calculate_expected_from_bool_map(lemon::ListGraph::EdgeMap<bool> & bool_map);
    

    // Methode, die true returned, falls das adden der Edge 'e' dazu fuehrt, dass es zusammen mit den edges, die in 'present_edges_map' true sind mindestens ein loop im Graphen gibt
    bool edge_creates_loop(const lemon::ListGraph::EdgeMap<bool> & present_edges_map, const lemon::ListGraph::Edge & e);
};

class FullyConnectedTwoStageMST : public TwoStageProblem {

    const unsigned int numberNodes;
    virtual void initialise_graph() override;

public:
    
    FullyConnectedTwoStageMST(unsigned int number_nodes, std::vector<double> & first_stage_weights, std::vector<std::vector<double>> & second_stage_weights, std::vector<double> & scenario_probabilites);
    virtual ~FullyConnectedTwoStageMST() = default;
};

// eine Klasse, die dazu da ist, einen bereits bestehenden Graph (mit nodes und edges liste) in ein TwoStageMST Problem einzubetten
class UseExternGraphTwoStageMST : public TwoStageProblem {

    virtual void initialise_graph() override;

public:
    UseExternGraphTwoStageMST(const lemon::ListGraph & _g, const std::vector<lemon::ListGraph::Node> & _nodes, const std::vector<lemon::ListGraph::Edge> & edges, const std::vector<double> & first_stage_weights, const std::vector<std::vector<double>> & second_stage_weights, std::vector<double> & scenario_probabilites);
    virtual ~UseExternGraphTwoStageMST() = default;

};