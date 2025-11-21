#pragma once

#include <map>
#include <set>
#include <vector>
#include <unordered_set>

#include "../core/instance.h"
#include "../core/solution.h"
#include "../util/canvas.h"


class DeterministicConstruction : public GuiElement
{
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
	};

	std::multimap<dist_t, Driver> busy_drivers_;
	std::vector<Driver> finished_drivers_;
	std::unordered_set<Node*> unvisited_pickups_;
	size_t accepted_requests_ = 0;
	size_t served_requests_ = 0;

	struct
	{
		int solution_index_ = 0;
		Canvas canvas_;
	} gui;

	void reset();

public:
	DeterministicConstruction(Instance* instance, bool keep_history = false);

	void step();
	void run_to_completion();

	Solution get_current_solution();
	bool done() const;

	void draw() override;
};