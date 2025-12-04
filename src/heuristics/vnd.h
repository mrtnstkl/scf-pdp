#pragma once

#include "core/algorithm.h"
#include "core/instance.h"
#include "core/solution.h"
#include "heuristics/localsearch.h"


struct VariableNeighborhoodDescent : public LocalSearch
{
	VariableNeighborhoodDescent(Instance* instance);
	size_t step() override;
	void reset() override;
	void draw() override;
};
