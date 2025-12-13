#pragma once

#include "core/algorithm.h"
#include "core/instance.h"
#include "core/solution.h"
#include "heuristics/localsearch.h"

struct Grasp : public Algorithm
{
	Instance* instance;
	Algorithm* randomized_construction;
	LocalSearch* local_search;

	struct Metrics : public SolutionMetrics
	{
		size_t iterations_before_improvement;
		void draw() override;
		void print_head(std::ostream& os) const override;
		void print(std::ostream& os) const override;
	} metrics;
	Solution current_best;
	float best_cost;

	StoppingCriterion stopping_criterion;
	bool finished = false;

	Grasp(Instance* instance, Algorithm* randomized_construction, LocalSearch* local_search);

	size_t step() override;
	void reset() override;
	bool done() override;
	Solution get_current_solution() override;
	void draw() override;
};
