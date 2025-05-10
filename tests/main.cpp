#include <cmath>
#include <iostream>

#include "bencher.hpp"

int main() {
    bch::Bencher bench{100'000, 1'000};

    std::cout << "Enter value" << std::endl;
    float val = 0;
    std::cin >> val;

    auto stats = bench.Run(pow, val, val);
    stats.Stat();
}
