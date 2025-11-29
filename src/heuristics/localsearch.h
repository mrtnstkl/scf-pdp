#pragma once

#include <string>
#include "core/algorithm.h"
#include "core/instance.h"
#include "core/solution.h"

struct LocalSearch : public Algorithm
{
	Instance* instance;

	// step should do a VND
	size_t step() override;
	// reset to initial solution
	void reset() override;
	// check if done --> occurs if neighborhoods exhausted
	bool done() override;

	Solution get_current_solution() override;

	// draw laod button
	void draw() override;

	void load_solution_from_file(const std::string& path);

	using Metrics = SolutionMetrics;
	Metrics metrics;

	Solution initial_solution_;
	Solution current_solution_;

	LocalSearch(Instance* instance);

	enum NeighborhoodType
	{
		MoveRequestWithinRouteNeighborhood,
		MoveRequestBetweenRoutesNeighborhood,
		SwapRequestsBetweenRoutesNeighborhood
	};

private:
	size_t k_ = 0;
	bool improved_ = false;
	bool apply_relocate(Solution& sol);
	bool apply_swap(Solution& sol);
	bool apply_two_opt(Solution& sol);

	bool is_route_feasible(const Route& route) const;
	bool is_solution_feasible(const Solution& sol) const;
};