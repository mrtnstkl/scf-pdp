#pragma once

#include <memory>

#include "core/algorithm.h"
#include "core/instance.h"
#include "core/solution.h"
#include "heuristics/neighborhood.h"
#include "neighborhood.h"

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
		// MOVE_REQUEST_WITHIN_ROUTE, MOVE_REQUEST_TO_ANOTHER_ROUTE, SWAP_REQUESTS_BETWEEN_ROUTES, // 20333
		// MOVE_REQUEST_WITHIN_ROUTE, SWAP_REQUESTS_BETWEEN_ROUTES, MOVE_REQUEST_TO_ANOTHER_ROUTE, // 22634
		// MOVE_REQUEST_TO_ANOTHER_ROUTE, MOVE_REQUEST_WITHIN_ROUTE, SWAP_REQUESTS_BETWEEN_ROUTES, // 17699
		MOVE_REQUEST_TO_ANOTHER_ROUTE, SWAP_REQUESTS_BETWEEN_ROUTES, MOVE_REQUEST_WITHIN_ROUTE, // 17130
		// SWAP_REQUESTS_BETWEEN_ROUTES, MOVE_REQUEST_WITHIN_ROUTE, MOVE_REQUEST_TO_ANOTHER_ROUTE, // 17546
		// SWAP_REQUESTS_BETWEEN_ROUTES, MOVE_REQUEST_TO_ANOTHER_ROUTE, MOVE_REQUEST_WITHIN_ROUTE, // 20058
		ENUM_COUNT,
	};

protected:
	Instance* instance_;
	Solution initial_solution_;
	Solution current_solution_;
	float current_cost_;
	bool done_;

public:
	ImprovementStrategy improvement_strategy;
	NeighborhoodType neighborhood_type;

	struct Metrics : public SolutionMetrics
	{
		int neighborhood_type;
		void draw() override;
		void print_head(std::ostream& os) const override;
		void print(std::ostream& os) const override;
	} metrics;

	LocalSearch(Instance* instance);

	size_t step() override;
	void reset() override;
	bool done() override;

	void set_initial_solution(const Solution& soln);

	Solution get_current_solution() override;

	void draw() override;
};
