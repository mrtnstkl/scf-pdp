#pragma once

#include <memory>

#include "core/algorithm.h"
#include "core/instance.h"
#include "core/solution.h"
#include "heuristics/neighborhood.h"

class LocalSearch : public Algorithm
{
public:
	enum ImprovementStrategy : int
	{
		FIRST_IMPROVEMENT = 0,
		BEST_IMPROVEMENT,
	};
	enum NeighborhoodType : int
	{
		DUMMY_NEIGHBORHOOD = 0,
		MOVE_REQUEST_WITHIN_ROUTE,
	};

private:
	Instance* instance_;
	Solution initial_solution_;
	Solution current_solution_;
	float current_cost_;
	bool done_;

	ImprovementStrategy improvement_strategy_;
	NeighborhoodType neighborhood_type_;

public:
	using Metrics = SolutionMetrics;
	Metrics metrics;

	LocalSearch(Instance* instance);

	size_t step() override;
	void reset() override;
	bool done() override;

	Solution get_current_solution() override;

	void draw() override;
};