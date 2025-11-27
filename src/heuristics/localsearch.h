#pragma once

#include "core/instance.h"
#include <string>
#include "neighborhoods.h"

struct LocalSearch : public Algorithm {
  	Instance *instance;
	
  	Solution current_solution_;
  	bool finished = false;
	
  	using Metrics = SolutionMetrics;
  	Metrics metrics;
	
  	size_t step() override;
  	void reset() override;
  	bool done() override;
  	Solution get_current_solution() override;

    void draw() override;

    LocalSearch(Instance *instance);

  	void load_solution_from_file(const std::string& path);

};
