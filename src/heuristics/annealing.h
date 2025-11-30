#pragma once

#include <random>
#include <set>
#include <vector>


#include "core/algorithm.h"
#include "core/instance.h"
#include "core/intermediary.h"
#include "core/solution.h"


struct Annealing : public Algorithm
{
	Instance* instance;
	Solution current_solution;
	float current_cost;
	float initial_temperature = 5.f;
	float cooling_rate = 0.99f;
	float min_temperature = 0;

	std::mt19937 rng;
	std::uniform_real_distribution<float> uniform_dist{0.f, 1.f};

	Annealing(Instance* instance);

	size_t step() override;
	void reset() override;
	bool done() override;

	Solution get_current_solution() override;

	using Metrics = SolutionMetrics;
	Metrics metrics;

	void draw() override;
};
