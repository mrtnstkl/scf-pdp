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
	using Driver = ::Driver;
	struct State
	{
		size_t step;
		bool done;
		size_t accepted_requests, served_requests;
		std::multimap<dist_t, GreedyConstruction::Driver> busy_drivers;
		std::vector<GreedyConstruction::Driver> finished_drivers;
		std::unordered_set<Node*> unvisited_pickups;
		State() = default;
		State(Solution& initial_solution);
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
	Solution initial_solution;
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
	GreedyNearestAny(Instance* instance);
	GreedyNearestAny(Instance& instance, const Solution& initial_solution);

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

struct GreedyInsertion : public Algorithm
{
	struct State
	{
		size_t step;
		bool done;
		size_t served_requests;
		// a route is a list of pairs of (node, load after visiting node)
		using Route = std::list<std::pair<Node*, int>>;
		std::vector<Route> routes;
		struct NearbyNode
		{
			std::vector<Route>::iterator route_it;
			Route::iterator node_it;
		};
		struct NodeOrder
		{
			bool operator()(const CustomerRequest* a, const CustomerRequest* b) const;
		};
		std::map<CustomerRequest*, std::map<dist_t, NearbyNode>, NodeOrder> unserved_requests;
		DeltaCalculator delta;
		State(Solution& initial_solution)
			: delta(initial_solution) {}
	};

	struct Metrics : public SolutionMetrics
	{
		size_t served_requests;
		void draw() override;
		void print_head(std::ostream& os) const override;
		void print(std::ostream& os) const override;
	};

	Instance* instance;
	Solution initial_solution;

	State state;
	Metrics metrics;

	GreedyInsertion(Instance* instance);
	GreedyInsertion(Instance* instance, const Solution& initial_solution);

	Solution get_current_solution() override;
	size_t step() override;
	bool done() override;
	virtual void reset();
	void draw() override;

};
