#pragma once

#include <map>
#include <set>
#include <vector>
#include <unordered_set>

#include "../core/instance.h"
#include "../core/solution.h"
#include "../util/canvas.h"


class ConstructionHeuristic : public GuiElement
{
protected:
	Instance* instance_;
	size_t step_ = 0;
	bool done_ = false;
	bool record_history_;
	std::vector<Solution> history_;

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

	std::multimap<dist_t, Driver> busy_drivers_;
	std::vector<Driver> finished_drivers_;
	std::unordered_set<Node*> unvisited_pickups_;
	size_t accepted_requests_ = 0;
	size_t served_requests_ = 0;

	struct
	{
		const char* name_ = "Base Construction Heuristic";
		int solution_index_ = 0;
		Canvas canvas_;
	} gui;

	void reset();

	virtual void step() = 0;

public:
	ConstructionHeuristic(Instance* instance, bool keep_history, const char* name);

	void compute_next();
	void run_to_completion();

	Solution get_current_solution();
	bool done() const;

	void draw() override;
};


class RandomConstruction : public ConstructionHeuristic
{
	void step() override;
public:
	RandomConstruction(Instance* instance, bool keep_history = false)
		: ConstructionHeuristic(instance, keep_history, "Random Construction Heuristic") {}
};

class DeterministicConstructionHeuristic : public ConstructionHeuristic
{
	void step() override;
public:
	DeterministicConstructionHeuristic(Instance* instance, bool keep_history = false)
		: ConstructionHeuristic(instance, keep_history, "Deterministic Construction Heuristic") {}
};