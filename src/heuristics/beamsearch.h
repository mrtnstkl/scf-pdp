#pragma once

#include <vector>
#include <set>

#include "core/instance.h"
#include "core/solution.h"
#include "core/algorithm.h"
#include "core/intermediary.h"


struct BeamSearch : public Algorithm
{
	struct TreeNode
	{
		bool finished = false;
		size_t step = 0;
		size_t accepted_requests = 0, completed_requests = 0;
		std::set<Node*> unvisited_pickups;
		std::vector<Driver> drivers;
		DeltaCalculator delta_calculator;

		float heuristic_cost() const;

		TreeNode(Instance* instance);
	};

	Instance* instance;
	std::map<float, TreeNode> beam;
	std::pair<float, TreeNode> best_node;
	bool finished = false;

	using Metrics = SolutionMetrics;
	Metrics metrics;

	int beam_width = 5;
	BeamSearch(Instance* instance);

	size_t step() override;
	void reset() override;
	bool done() override;

	void draw() override;

	Solution get_current_solution() override;
};