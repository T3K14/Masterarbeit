#include <random>
#include <ctime>
#include <iostream>

int main() {

    std::random_device seeder;
    const auto seed = seeder.entropy() ? seeder() : time(nullptr);
    std::cout << seed << std::endl;

    std::mt19937 eng(static_cast<std::mt19937::result_type>(seed));
    std::uniform_int_distribution<int> dist(1, 99);

    for (int i=0; i<10; i++) {
        std::cout << dist(eng) << "\n";
    }
}