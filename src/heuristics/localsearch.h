#pragma once

#include <list>
#include <queue>
#include <vector>
#include "string"


#include "core/algorithm.h"
#include "core/instance.h"
#include "core/solution.h"

struct LocalSearch : public Algorithm
{
	Instance* instance;

	size_t step() override;
	void reset() override;
	bool done() override;

	Solution get_current_solution() override;

	void draw() override;

	void load_solution_from_file(const std::string& path);
	using Metrics = SolutionMetrics;
	Metrics metrics;
	Solution current_solution_;

	LocalSearch(Instance* instance);
};