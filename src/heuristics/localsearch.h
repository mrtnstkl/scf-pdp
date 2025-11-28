#pragma once

#include <string>
#include "core/instance.h"
#include "memory"
#include "neighborhoods.h"


// Is to be a Variable Neighborhood Descent (VND) algorithm
struct LocalSearch : public Algorithm
{
public:
	explicit LocalSearch(Instance* instance);

	size_t step() override;
	void reset() override;
	bool done() override;
	Solution get_current_solution() override;
	void draw() override;
	void load_solution_from_file(const std::string& path);

	using Metrics = SolutionMetrics;
	Metrics metrics;

private:
	Instance* instance = nullptr;
	Solution current_solution_;
	bool finished = false;
	size_t external_step_counter_ = 0;

	enum class NeighborhoodType
	{
		MoveWithinRoute,
		SwapBetweenVehicles,
		MoveBetweenVehicles
	};

	float current_cost_ = std::numeric_limits<float>::infinity();


	NeighborhoodType selected_neighborhood_ = NeighborhoodType::MoveBetweenVehicles;
	std::unique_ptr<Neighborhood> neighborhood_;
};
