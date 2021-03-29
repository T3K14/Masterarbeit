#include "utilities.h"
#include <random>
#include <ctime>

#include <iostream>

std::vector<double> calcScenarioProbabilities(size_t n) {

    const auto seed = time(nullptr);
    std::mt19937 eng(static_cast<std::mt19937::result_type>(seed));

    std::uniform_real_distribution<double> dist(1, 100);

    double sum = 0;
    std::vector<double> vec;
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

int main() {

    double n = 5;
    auto res = calcScenarioProbabilities(n);
    for (int i=0; i<n; i++) {
        std::cout << res[i] << ", ";
    }
    std::cout << "\n";
}