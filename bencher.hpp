#ifndef BENCHER_HPP_
#define BENCHER_HPP_

#include <cstddef>
#include <cstring>
#include <iostream>
#include <utility>
#include <functional>
#include <algorithm>
#include <limits>
#include <cmath>
#include <type_traits>
#include <atomic>
#include <stdexcept>
#include <string>

#include <x86intrin.h>
#include <sched.h>
#include <unistd.h>

namespace bch {

namespace detail {

// Force the compiler to flush pending writes to global memory. Acts as an
// effective read/write barrier
// from https://github.com/google/benchmark/blob/main/include/benchmark/benchmark.h
inline void ClobberMemory() {
  std::atomic_signal_fence(std::memory_order_acq_rel);
}

// The DoNotOptimize(...) function can be used to prevent a value or
// expression from being optimized away by the compiler. This function is
// intended to add little to no overhead.
// See: https://youtu.be/nXaxk27zwlk?t=2441
// from https://github.com/google/benchmark/blob/main/include/benchmark/benchmark.h
template <class T>
inline void DoNotOptimize(T& value) {
#if defined(__clang__)
  asm volatile("" : "+r,m"(value) : : "memory");
#else
  asm volatile("" : "+m,r"(value) : : "memory");
#endif
}

template <class T>
inline void DoNotOptimize(T&& value) {
#if defined(__clang__)
  asm volatile("" : "+r,m"(value) : : "memory");
#else
  asm volatile("" : "+m,r"(value) : : "memory");
#endif
}

template <class T>
inline void DoNotOptimize(const T& value) {
  asm volatile("" : : "r,m"(value) : "memory");
}

// Invokes function with args and prevent optimization
template <typename Func, typename... Args>
inline void VoidInvokeNoOpt(Func&& func, Args&&... args) {
    if constexpr (std::is_void_v<decltype(func(std::forward<Args>(args)...))>) {
        std::invoke(std::forward<Func>(func), std::forward<Args>(args)...);
        detail::ClobberMemory();
    } else {
        auto res = std::invoke(std::forward<Func>(func), std::forward<Args>(args)...);
        detail::DoNotOptimize(res);
    }
}

} // namespace detail

class BenchStats {
  public:
    constexpr static size_t kTreshholdMultiplier = 10;
  // private:
  public:
    size_t elem_per_call_;
    // size_t n_stable_calls_;
    // size_t n_unstable_calls_;
    // clock per call = latency
    double mean_clock_per_call_;
    double variance_clock_per_call_;

    size_t max_clock_per_call_;
    size_t min_clock_per_call_;
  public:
    explicit BenchStats(size_t elem_per_call = 1) 
        :
        elem_per_call_{elem_per_call},
        // n_stable_calls_{0},
        // n_unstable_calls_{0},
        mean_clock_per_call_{0}, 
        variance_clock_per_call_{0}, 
        max_clock_per_call_{std::numeric_limits<size_t>::min()}, 
        min_clock_per_call_{std::numeric_limits<size_t>::max()}
    {}

    template <typename CharT = char>
    void Stat(std::basic_ostream<CharT>& ostm = std::cout) {
        ostm << "Benchmarking statistics:" << std::endl;

        ostm << "latency per call: " << mean_clock_per_call_ << " +- " << std::sqrt(variance_clock_per_call_) << std::endl;
        ostm << "max latency per call: " << max_clock_per_call_ << std::endl;
        ostm << "min latency per call: " << min_clock_per_call_ << std::endl;

        double epc = static_cast<double>(elem_per_call_);
        double mean_throughput = epc / mean_clock_per_call_;
        double varience_throughput = epc / std::sqrt(variance_clock_per_call_);
        double max_throughput = epc / static_cast<double>(min_clock_per_call_);
        double min_throughput = epc / static_cast<double>(max_clock_per_call_);
        
        ostm << "throughput per call: " << mean_throughput << " +- " << varience_throughput << std::endl;
        ostm << "max throughput per call: " << max_throughput << std::endl;
        ostm << "min throughput per call: " << min_throughput << std::endl;
    }
};

class Bencher {
  private:
    static constexpr size_t kZeroFuncCallLat = 84;

    size_t elem_per_call_;
    size_t num_iters_;
    size_t num_warmups_;
  public:
    explicit Bencher(size_t num_iters = 1, size_t num_warmups = 0, size_t elem_per_call = 1) 
        : 
        elem_per_call_{elem_per_call},
        num_iters_{num_iters}, 
        num_warmups_{num_warmups}
    {}
 
    void BindToCore(int core_ind) {
        cpu_set_t cpu_set;
        CPU_ZERO(&cpu_set);
        // 0 means current thread
        CPU_SET(0, &cpu_set);
        auto res = sched_setaffinity(core_ind, sizeof(cpu_set), &cpu_set); 
        if (res != 0) {
            throw std::runtime_error("Cant bind to core: " + std::string{std::strerror(errno)});
        }       
    }

    void UpPriority() {           
        errno = 0;
        auto res = nice(-20);
        if (res != 0 && errno != 0) {
            throw std::runtime_error("Cant set priority: " + std::string{std::strerror(errno)});
        }
    }

    template <typename Func, typename... Args>
    BenchStats Run(Func&& func, Args&&... args) {
        BenchStats stats{elem_per_call_};

        for (size_t i = 0; i < num_warmups_; i++) {
            // Warmup runs
            detail::VoidInvokeNoOpt(func, args...);
        }

        for (size_t i = 0; i < num_iters_; i++) {
            size_t start = __rdtsc(); 
            _mm_lfence();

            detail::VoidInvokeNoOpt(func, args...);

            _mm_mfence();
            _mm_lfence();
            size_t end = __rdtsc(); 

            size_t latency = end - start - kZeroFuncCallLat;
            UpdateMeasures(&stats, latency, i);
        }

        return stats;
    }

  private:
    void UpdateMeasures(BenchStats* stats, size_t new_lat, size_t current_ind) {
        stats->max_clock_per_call_ = std::max(new_lat, stats->max_clock_per_call_);
        stats->min_clock_per_call_ = std::min(new_lat, stats->min_clock_per_call_);

        double current_mean = stats->mean_clock_per_call_;
        double current_var = stats->variance_clock_per_call_;

        double x = static_cast<double>(new_lat);
        double i = static_cast<double>(current_ind);

        // one pass mean and varience calculation
        double next_mean = current_mean + (x - current_mean) / (i + 1);
        double next_var = current_var + ((x - current_mean) * (x - next_mean) - current_var) / (i + 1);
        
        stats->mean_clock_per_call_ = next_mean;
        stats->variance_clock_per_call_ = next_var;
    }
};

} // namespace bch

#endif // BENCHER_HPP_
