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

	struct Metrics : SolutionMetrics
	{
		size_t iterations_before_improvement;
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


template<>
inline void draw<Grasp::Metrics>(Grasp::Metrics& drawable)
{
	SolutionMetrics& base = drawable;
	::draw(base);
	ImGui::Text("Iterations Before Improvement: %zu", drawable.iterations_before_improvement);
}
