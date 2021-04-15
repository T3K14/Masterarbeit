#include "utilities.hpp"
#include <random>
#include <ctime>

// #include <iostream>

// define the RNG
std::mt19937 rng{std::random_device{}()};               // construct it with the seed that I get from the random_device rvalue on which I use the ()-operator (functor)


std::vector<double> calcScenarioProbabilities(size_t n) {

    const auto seed = time(nullptr);
    std::mt19937 eng(static_cast<std::mt19937::result_type>(seed));

    std::uniform_real_distribution<double> dist(1, 100);                // Intervall anders machen? 

    double sum = 0;
    std::vector<double> vec;
    vec.reserve(n);
    for (int i=0; i<n; i++) {
        double res = dist(eng);
        vec.push_back(res);
        sum += res;
    }

    for (int i=0; i<n; i++) {
        vec[i] /= sum;
    }

    return vec;
} 

// int main() {

//     double n = 5;
//     auto res = calcScenarioProbabilities(n);
//     for (int i=0; i<n; i++) {
//         std::cout << res[i] << ", ";
//     }
//     std::cout << "\n";
// }