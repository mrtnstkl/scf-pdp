#pragma once

#include <map>
#include <ostream>
#include <unordered_set>
#include <vector>

#include "core/algorithm.h"
#include "core/instance.h"
#include "core/intermediary.h"
#include "core/solution.h"

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
	struct Metrics : public SolutionMetrics
	{
		size_t accepted_requests, served_requests;
		std::vector<int> driver_loads;
		void draw() override;
		void print_head(std::ostream& os) const override;
		void print(std::ostream& os) const override;
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
	int tightening_interval = 5;

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