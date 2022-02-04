#pragma once

// #include "two_stage_gurobi.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"
#include <random>

enum class Vergleich {ApproxVs4b, ApproxVsTriv, ApproxVsBruteforce};

// forward declaration  KLAPPT IRGENDWIE NICHT
// class TwoStageProblem;
// double solve_relaxed_lp(TwoStageProblem & two_stage_problem);

template <typename... Tn>
void simulate(int i, Tn..., double d);

class ScenarioCreator {

    private:

    public:
        virtual ~ScenarioCreator() = default;
        virtual void create_scenarios(TwoStageProblem & tsp) = 0;

    protected:
        void override_scenarios(TwoStageProblem & tsp, std::vector<double> & scenario_probabilites);
    
};

class NRandomScenarioCreator : public ScenarioCreator {

    private:
        unsigned int number_scenarios;
        std::mt19937 & rng;

    public:

        virtual ~NRandomScenarioCreator() = default;

        NRandomScenarioCreator(unsigned int _number_scenarios, std::mt19937 & _rng);

        virtual void create_scenarios(TwoStageProblem & tsp) override;

};

class NewEdgeCostCreator {

    private:
        // RNG & rng;
        // std::uniform_real_distribution<double> uniformDist;

    public:
        virtual ~NewEdgeCostCreator() = default;
        virtual void create_costs(TwoStageProblem & tsp) = 0;

        // virtual void delete_costs(); // ?? brauche ich vllt, um die alten Maps zu leeren
    protected:
        void override_costs(TwoStageProblem & tsp, std::vector<double> & first_stage_costs, std::vector<std::vector<double>> & second_stage_costs);

};

class RandomTestCreator : public NewEdgeCostCreator {

    double low;
    double high;
    std::mt19937 & rng;

    public:

        virtual ~RandomTestCreator() = default;

        RandomTestCreator(double _low, double _high, std::mt19937 & _rng);

        virtual void create_costs(TwoStageProblem & tsp) override;

};

// vllt ist ensemble auch falsche Bezeichnung
class Ensemble {

// protected:
public:
    unsigned int number_nodes;
    TwoStageProblem two_stage_problem;
    NewEdgeCostCreator & edge_cost_creator;
    ScenarioCreator & secenario_creator;

    virtual void erase_all_edges();
    // virtual void add_edges() = 0;        Das mache ich in allen Childklassen individuell

public:

    // will ich nen default constructor?

    virtual ~Ensemble() = default;

    // constructor
    Ensemble(unsigned int number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator);

    // soll das Ensemble neu aufsetzen, zB. einen neuen Baum erzeugen und neue Kosten
    virtual void recreate();// = 0;      
    virtual void add_edges() = 0;       // PROTECTED MACHEN? SONST KANN ICH DIE AUCH VON AUSSERHALB AUFRUFEN, ODER SIND DIE SAVE SO???                  
    virtual void initialize();

    void approx_after_lp(std::mt19937 & rng);
    double bruteforce();

    // bei 4b kommt keine 1.stage Kantenauswahl raus, daher wird hier direkt das Ergebnis returned
    double do4b();
    void greedy();

    // speichert mir den aktuellen graphen des twostageproblems
    virtual void save_current_graph(std::string name);

    friend double solve_relaxed_lp(TwoStageProblem & two_stage_problem);

};


class Tree : public Ensemble {

protected:
    std::mt19937 & rng;

public:
    Tree(unsigned int number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator, std::mt19937 & _rng);

    virtual ~Tree() = default;
    // virtual void recreate() override;               // was passiert mit dieser methode???, wird die versteckt?
    // virtual void recreate() override;

    virtual void add_edges() override;

};

// Ensemble-Klasse, die zuerst einen Tree baut und dann noch random N Edges zum Graph hinzufuegt
class TreePlusEdges : public Tree {

protected:
    unsigned int number_extra_edges;

public:
    TreePlusEdges(unsigned int _number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator, std::mt19937 & _rng, unsigned int _number_extra_edges);

    virtual ~TreePlusEdges() = default;
    virtual void add_edges() override;


};

class FullyConnected : public Ensemble {

public:
    FullyConnected(unsigned int number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator);
    virtual ~FullyConnected() = default;
    virtual void add_edges() override;

};

class FullyConnectedMinusEdges : public FullyConnected {

protected:
    unsigned int number_minus_edges;
    std::mt19937 & rng;

public:
    FullyConnectedMinusEdges(unsigned int _number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator, std::mt19937 & _rng, unsigned int _number_minus_edges);
    virtual ~FullyConnectedMinusEdges() = default;
    virtual void add_edges() override;

};


void simulate(unsigned int runs, Ensemble & ensemble, Vergleich vergleich);

