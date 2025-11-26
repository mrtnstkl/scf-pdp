#pragma once


#include "core/instance.h"
#include "neighborhoods.h"


struct LocalSearch : public Algorithm
{
	Instance* instance;


	size_t step() override {return 0;};
	void reset() override{};
	bool done() override{return false;};
	
	Solution get_current_solution() override {return Solution(instance);};

	void draw() override {};

	LocalSearch(Instance* instance);
	~LocalSearch() override = default;

};