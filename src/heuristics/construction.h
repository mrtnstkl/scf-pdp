#pragma once

#include <map>
#include <set>
#include <vector>
#include <unordered_set>

#include "../core/instance.h"
#include "../core/solution.h"
#include "../core/algorithm.h"
#include "../util/canvas.h"


struct ConstructionHeuristic : public Algorithm
{
	struct Driver
	{
		int id;
		dist_t time = 0;
		int load = 0;
		std::set<CustomerRequest*> loaded_requests;
		std::vector<Node*> path;

		Driver(int id = 0) : id(id) {}

		void visit(Node* node, Instance& instance);
		bool fits(Node* node, Instance& instance) const;
	};
	struct State
	{
		size_t step;
		bool done;
		size_t accepted_requests, served_requests;
		std::multimap<dist_t, ConstructionHeuristic::Driver> busy_drivers;
		std::vector<ConstructionHeuristic::Driver> finished_drivers;
		std::unordered_set<Node*> unvisited_pickups;
	};
	struct Metrics : SolutionMetrics
	{
		size_t step;
		size_t accepted_requests, served_requests;
		std::vector<int> driver_loads;
	};

	Instance* instance;
	State state;
	Metrics metrics;

	ConstructionHeuristic(Instance* instance);

	Solution get_current_solution() override;
	bool done() override;
	void draw() override { };
	virtual void reset();

	virtual size_t step() = 0;
};

template<>
inline void draw<ConstructionHeuristic::Metrics>(ConstructionHeuristic::Metrics& drawable)
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


struct RandomConstruction : public ConstructionHeuristic
{
	RandomConstruction(Instance* instance)
		: ConstructionHeuristic(instance) {}

	size_t step() override;
};

struct GreedyNearestAny : public ConstructionHeuristic
{
	GreedyNearestAny(Instance* instance)
		: ConstructionHeuristic(instance) {}

	size_t step() override;
};

struct GreedyNearestWeighted : public ConstructionHeuristic
{
	float mean_distance_;
	float dropoff_bias_ = 1.5f;
	
	GreedyNearestWeighted(Instance* instance);

	size_t step() override;
	void draw() override;
};