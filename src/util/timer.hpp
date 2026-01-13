#pragma once

#include <chrono>
#include <string>
#if __has_include(<format>)
#include <format>
#endif

class Timer
{
	std::chrono::steady_clock::time_point start_, stop_;
	bool running_;

public:
	Timer()
	{
		reset();
	}
	void reset()
	{
		start_ = std::chrono::steady_clock::now();
		running_ = true;
	}
	Timer& stop()
	{
		running_ = false;
		stop_ = std::chrono::steady_clock::now();
		return *this;
	}
	template <typename Duration, typename F = float>
	auto get_fp()
	{
		const std::chrono::duration<F, Duration> d =
			(running_ ? std::chrono::steady_clock::now() : stop_) - start_;
		return d.count();
	}
	template <typename Duration = std::string>
	auto get()
	{
		using namespace std::chrono;
		const auto duration = (running_ ? steady_clock::now() : stop_) - start_;
		if constexpr (std::is_same_v<Duration, std::string>)
		{
			using namespace std::chrono_literals;
			if (duration < 10'000ns)
				return std::to_string(duration_cast<nanoseconds>(duration).count()) + "ns";
			if (duration < 10'000us)
				return std::to_string(duration_cast<microseconds>(duration).count()) + "us";
			if (duration < 10'000ms)
				return std::to_string(duration_cast<milliseconds>(duration).count()) + "ms";
			return std::to_string(duration_cast<seconds>(duration).count()) + "s";
		}
		else
		{
			return (uint64_t) duration_cast<Duration>(duration).count();
		}
	}
	template <typename Duration = std::string>
	auto get_and_reset()
	{
		auto v = get<Duration>();
		reset();
		return v;
	}
};

inline std::string timestamp(std::chrono::system_clock::time_point time = std::chrono::system_clock::now())
{
#ifdef __cpp_lib_format
	return std::format("{0:%Y%m%d%H%M%S}", time);
#else
	const std::time_t t = std::chrono::system_clock::to_time_t(time);
	const std::tm* tm = std::localtime(&t);
	char buf[16];
	std::strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", tm);
	return buf;
#endif
}
