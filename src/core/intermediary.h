#pragma once

#include <vector>
#include <set>
#include <unordered_set>
#include <map>

#include "core/instance.h"
#include "core/solution.h"
#include "core/algorithm.h"

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