#pragma once

#include "core/algorithm.h"
#include "core/instance.h"
#include "core/solution.h"

struct LocalSearch : public Algorithm
{
	Instance* instance;


	size_t step() override;
	void reset() override;
	bool done() override;

	Solution get_current_solution() override;

	void draw() override;
};