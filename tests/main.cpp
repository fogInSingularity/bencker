#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>

#include "bencher.hpp"

constexpr std::string_view kRedColor = "\x1b[31m";
constexpr std::string_view kBoldStyle = "\x1b[1m";
constexpr std::string_view kResetColor = "\x1b[0m";

// empty func
void foo() {}

int main() {
    bch::Bencher bench{1'000'000, 10'000};
    try {
        bench.BindToCore(3);
    } catch (const std::runtime_error& e) {
        std::cerr << kRedColor << kBoldStyle << e.what() << kResetColor << std::endl;
    }

    try {
        bench.UpPriority();
    } catch (const std::runtime_error& e) {
        std::cerr << kRedColor << kBoldStyle << e.what() << kResetColor << std::endl;
    }

    std::cout << "Enter value" << std::endl;
    float val = 0;
    std::cin >> val;

    auto stats = bench.Run(pow, val, val);
    // auto stats = bench.Run(foo);
    stats.Stat();
}
