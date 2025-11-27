#pragma once

#include "core/instance.h"
#include <string>
#include "neighborhoods.h"
#include "memory"




struct LocalSearch : public Algorithm {
  	Instance *instance;

  	Solution current_solution_;
  	bool finished = false;

	size_t external_step_counter_ = 0;
		
	enum class NeighborhoodType {
		MoveWithinRoute,
		SwapBetweenVehicles,
		MoveBetweenVehicles
	};

	NeighborhoodType selected_neighborhood_ = NeighborhoodType::MoveBetweenVehicles;
	std::unique_ptr<Neighborhood> neighborhood_;

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
