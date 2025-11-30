#include "heuristics/localsearch.h"

#include <filesystem>

#include "imgui.h"


LocalSearch::LocalSearch(Instance* instance)
	: instance_(instance), initial_solution_(instance, true), current_solution_(initial_solution_)
{
	if (std::filesystem::exists("solutions/current_solution.sln"))
		initial_solution_ = Solution::load_from_file("solutions/current_solution.sln", instance);
	reset();
}

size_t LocalSearch::step()
{

	std::unique_ptr<Neighborhood> neighborhood;
	switch (neighborhood_type_)
	{
	case DUMMY_NEIGHBORHOOD:
		neighborhood = std::make_unique<DummyNeighborhood>(current_solution_);
		break;
	case MOVE_REQUEST_WITHIN_ROUTE:
		neighborhood = std::make_unique<MoveRequestWithinRouteNeighborhood>(current_solution_, instance_);
		break;
	case MOVE_REQUEST_TO_ANOTHER_ROUTE:
		neighborhood = std::make_unique<MoveRequestToAnotherRouteNeighborhood>(current_solution_, instance_);
		break;
	}
	

	bool improvement_found = false;
	if (improvement_strategy_ == FIRST_IMPROVEMENT)
	{
		while (neighborhood->has_next())
		{
			const float next_cost = neighborhood->next();
			if (next_cost >= current_cost_)
				continue;
			current_cost_ = next_cost;
			current_solution_ = neighborhood->get();
			improvement_found = true;
			break;
		}
	}
	else if (improvement_strategy_ == BEST_IMPROVEMENT)
	{
		while (neighborhood->has_next())
		{
			const float next_cost = neighborhood->next();
			if (next_cost > current_cost_)
				continue;
			current_cost_ = next_cost;
			current_solution_ = neighborhood->get();
			improvement_found = true;
		}
	}
	
	if (improvement_found)
	{
		metrics.step++;
		current_solution_.calculate_metrics(metrics);
	}
	else
		done_ = true;

	return metrics.step;
}

void LocalSearch::reset()
{
	current_solution_ = initial_solution_;
	current_cost_ = current_solution_.calculate_cost();
	done_ = false;
	metrics.step = 0;
	current_solution_.calculate_metrics(metrics);
}

bool LocalSearch::done() { return done_; }

Solution LocalSearch::get_current_solution() { return current_solution_; }

void LocalSearch::draw()
{
	static const char* neighborhood_labels[]{"Dummy", "Move Request Within Route", "Move Request To Another Route"};
	ImGui::Combo("Neighborhood", reinterpret_cast<int*>(&neighborhood_type_), neighborhood_labels, 3);

	static const char* strat_labels[]{"First", "Best"};
	ImGui::Combo(
		"Improvement Strat", reinterpret_cast<int*>(&improvement_strategy_), strat_labels, 2);

	if (ImGui::Button("Load Solution from File"))
	{
		if (std::filesystem::exists("solutions/current_solution.sln"))
			initial_solution_ = Solution::load_from_file("solutions/current_solution.sln", instance_);
	}
}
