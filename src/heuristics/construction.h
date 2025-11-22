#pragma once

#include <map>
#include <set>
#include <vector>
#include <unordered_set>
#include <random>

#include "core/instance.h"
#include "core/solution.h"
#include "core/algorithm.h"
#include "core/intermediary.h"
#include "util/canvas.h"

struct GreedyConstruction : public Algorithm
{
	using Driver = Driver;
	struct State
	{
		size_t step;
		bool done;
		size_t accepted_requests, served_requests;
		std::multimap<dist_t, GreedyConstruction::Driver> busy_drivers;
		std::vector<GreedyConstruction::Driver> finished_drivers;
		std::unordered_set<Node*> unvisited_pickups;
	};
	struct Metrics : SolutionMetrics
	{
		size_t accepted_requests, served_requests;
		std::vector<int> driver_loads;
	};

	Instance* instance;
	State state;
	Metrics metrics;

	GreedyConstruction(Instance* instance);

	Solution get_current_solution() override;
	bool done() override;
	void draw() override { };
	virtual void reset();

	virtual size_t step() = 0;
};

template<>
inline void draw<GreedyConstruction::Metrics>(GreedyConstruction::Metrics& drawable)
{
	SolutionMetrics& base = drawable;
	draw(base);
	ImGui::Text("Step: %zu", drawable.step);
	ImGui::Text("Accepted Requests: %zu", drawable.accepted_requests);
	ImGui::Text("Served Requests: %zu", drawable.served_requests);
	for (size_t i = 0; i < drawable.driver_loads.size(); ++i)
	{
		ImGui::Text("Driver %zu Load: %d", i, drawable.driver_loads[i]);
	}
}


struct RandomConstruction : public GreedyConstruction
{
	RandomConstruction(Instance* instance)
		: GreedyConstruction(instance) {}

	size_t step() override;
};

struct GreedyNearestAny : public GreedyConstruction
{
	GreedyNearestAny(Instance* instance)
		: GreedyConstruction(instance) {}

	size_t step() override;
};

struct GreedyRandomizedClosest : public GreedyConstruction
{
	int candidates_count = 3;

	GreedyRandomizedClosest(Instance* instance)
		: GreedyConstruction(instance) {}

	size_t step() override;
	void draw() override;
};

struct GreedyNearestWeighted : public GreedyConstruction
{
	float mean_distance_;
	float dropoff_bias_ = 1.5f;
	
	GreedyNearestWeighted(Instance* instance);

	size_t step() override;
	void draw() override;
};