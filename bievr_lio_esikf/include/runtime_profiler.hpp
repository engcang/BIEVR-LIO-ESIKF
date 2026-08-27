#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

struct RuntimeStatistics
{
    std::size_t sample_count_ = 0U;
    double mean_ms_ = 0.0;
    double p50_ms_ = 0.0;
    double p95_ms_ = 0.0;
    double p99_ms_ = 0.0;
    double maximum_ms_ = 0.0;
};

class RuntimeProfiler
{
public:
    class Measurement
    {
    public:
        explicit Measurement(RuntimeProfiler &_profiler):
            profiler_(_profiler.enabled_ ? &_profiler : nullptr)
        {
            if (profiler_ != nullptr)
            {
                start_time_ = std::chrono::steady_clock::now();
            }
        }

        ~Measurement() = default;

        void finish()
        {
            if (profiler_ != nullptr)
            {
                const auto end_time = std::chrono::steady_clock::now();
                const double runtime_ms = std::chrono::duration<double, std::milli>(end_time - start_time_).count();
                profiler_->runtimes_ms_.push_back(runtime_ms);
                profiler_ = nullptr;
            }
        }

        Measurement(const Measurement &) = delete;
        Measurement &operator=(const Measurement &) = delete;
        Measurement(Measurement &&) = delete;
        Measurement &operator=(Measurement &&) = delete;

    private:
        RuntimeProfiler *profiler_ = nullptr;
        std::chrono::steady_clock::time_point start_time_;
    };

    void configure(const bool _enabled,
                   const std::string &_output_path)
    {
        enabled_ = _enabled;
        output_path_ = _output_path;
        runtimes_ms_.clear();
        if (enabled_)
        {
            runtimes_ms_.reserve(100000U);
        }
    }

    bool enabled() const
    {
        return enabled_;
    }

    const std::string &outputPath() const
    {
        return output_path_;
    }

    RuntimeStatistics statistics() const
    {
        RuntimeStatistics result;
        result.sample_count_ = runtimes_ms_.size();
        if (runtimes_ms_.empty())
        {
            return result;
        }

        result.mean_ms_ = std::accumulate(runtimes_ms_.begin(), runtimes_ms_.end(), 0.0) /
                          static_cast<double>(runtimes_ms_.size());
        std::vector<double> sorted_runtimes = runtimes_ms_;
        std::sort(sorted_runtimes.begin(), sorted_runtimes.end());
        result.p50_ms_ = percentile(sorted_runtimes, 0.50);
        result.p95_ms_ = percentile(sorted_runtimes, 0.95);
        result.p99_ms_ = percentile(sorted_runtimes, 0.99);
        result.maximum_ms_ = sorted_runtimes.back();
        return result;
    }

    std::string summary() const
    {
        const RuntimeStatistics result = statistics();
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(6)
               << "samples=" << result.sample_count_
               << ", mean=" << result.mean_ms_ << " ms"
               << ", p50=" << result.p50_ms_ << " ms"
               << ", p95=" << result.p95_ms_ << " ms"
               << ", p99=" << result.p99_ms_ << " ms"
               << ", max=" << result.maximum_ms_ << " ms";
        return stream.str();
    }

    bool writeCsv() const
    {
        if (!enabled_ || output_path_.empty())
        {
            return true;
        }

        std::ofstream output(output_path_, std::ios::out | std::ios::trunc);
        if (!output)
        {
            return false;
        }

        output << "scan_index,runtime_ms\n"
               << std::fixed << std::setprecision(9);
        for (std::size_t index = 0U; index < runtimes_ms_.size(); ++index)
        {
            output << index << ',' << runtimes_ms_[index] << '\n';
        }
        return static_cast<bool>(output);
    }

private:
    static double percentile(const std::vector<double> &_sorted_values,
                             const double _probability)
    {
        if (_sorted_values.size() == 1U)
        {
            return _sorted_values.front();
        }

        const double rank = _probability * static_cast<double>(_sorted_values.size() - 1U);
        const std::size_t lower_index = static_cast<std::size_t>(rank);
        const std::size_t upper_index = std::min(lower_index + 1U, _sorted_values.size() - 1U);
        const double upper_weight = rank - static_cast<double>(lower_index);
        return _sorted_values[lower_index] * (1.0 - upper_weight) +
               _sorted_values[upper_index] * upper_weight;
    }

    bool enabled_ = false;
    std::string output_path_;
    std::vector<double> runtimes_ms_;
};
