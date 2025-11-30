#include "heuristics/vnd.h"
#include "vnd.h"

#include <filesystem>
#include <iostream>

VariableNeighborhoodDescent::VariableNeighborhoodDescent(Instance* instance)
	: LocalSearch(instance)
{
	reset();
}

size_t VariableNeighborhoodDescent::step()
{

	std::unique_ptr<Neighborhood> neighborhood;

	switch (neighborhood_type_)
	{
	case MOVE_REQUEST_WITHIN_ROUTE:
		neighborhood =
			std::make_unique<MoveRequestWithinRouteNeighborhood>(current_solution_, instance_);
		break;
	case MOVE_REQUEST_TO_ANOTHER_ROUTE:
		neighborhood =
			std::make_unique<MoveRequestToAnotherRouteNeighborhood>(current_solution_, instance_);
		break;
	case SWAP_REQUESTS_BETWEEN_ROUTES:
		neighborhood =
			std::make_unique<SwapRequestsBetweenRoutesNeighborhood>(current_solution_, instance_);
		break;
	default:
		assert(false && "Unknown neighborhood type");
	}

	bool improvement_found = false;

	while (neighborhood->has_next())
	{
		const float next_cost = neighborhood->next();
		if (next_cost >= current_cost_)
			continue;
		current_cost_ = next_cost;
		current_solution_ = neighborhood->get();
		improvement_found = true;
	}

	if (improvement_found)
	{
		std::cout << "VND Improvement found in N_" << neighborhood_type_
				  << " (cost: " << current_cost_ << ")\n";
		metrics.step++;
		current_solution_.calculate_metrics(metrics);
		metrics.neighborhood_type = int(neighborhood_type_);
		neighborhood_type_ = NeighborhoodType(0); // restart with first neighborhood
	}
	else
	{
		// move to next neighborhood
		neighborhood_type_ = NeighborhoodType(int(neighborhood_type_) + 1);
		if (neighborhood_type_ == LocalSearch::NeighborhoodType::ENUM_COUNT)
			done_ = true; // completed all neighborhood types
	}

	return metrics.step;
}

void VariableNeighborhoodDescent::reset()
{
	LocalSearch::reset();
	neighborhood_type_ = LocalSearch::NeighborhoodType(0);
}

void VariableNeighborhoodDescent::draw()
{
	if (ImGui::Button("Load Solution from File"))
	{
		if (std::filesystem::exists("solutions/current_solution.sln"))
			initial_solution_ =
				Solution::load_from_file("solutions/current_solution.sln", instance_);
	}
}
