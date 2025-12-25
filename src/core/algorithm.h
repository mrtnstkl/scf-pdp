#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>

#include "core/solution.h"
#include "util/canvas.h"
#include "util/gui.h"
#include "util/timer.hpp"

struct Algorithm : public GuiElement
{
	struct Worker
	{
		std::atomic_bool running;
		std::atomic_bool stop_requested;
		std::mutex data_mutex;
		std::thread thread;
		void start(std::function<void(const std::atomic_bool&)> func);
		void stop();
	} worker;
	virtual ~Algorithm();

	virtual size_t step() = 0;
	virtual void reset() = 0;
	virtual bool done() = 0;

	virtual Solution get_current_solution() = 0;

	void draw() override = 0;

	static Solution run(Algorithm& algorithm);
};

struct StoppingCriterion : public GuiElement
{
	enum Type : int { TIME, COST, HITS, STAGNATION } type = STAGNATION;
	
	uint64_t time_limit_ms = 1000;
	Timer time_timer;
	bool time_running;

	float cost_target = 1000.f;

	size_t hits_limit = 100;
	size_t hits_current;

	size_t stagnation_limit = 100;
	size_t stagnation_counter;
	float stagnation_best = std::numeric_limits<float>::max();

	StoppingCriterion();

	bool should_stop(float current_cost);
	void reset();

	void draw() override;
};
