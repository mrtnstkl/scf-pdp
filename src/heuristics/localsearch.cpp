#include "heuristics/localsearch.h"

#include <filesystem>

#include "imgui.h"

void LocalSearch::Metrics::draw()
{
	SolutionMetrics::draw();
	ImGui::Text("Neighborhood Type: %d", neighborhood_type);
}

void LocalSearch::Metrics::print_head(std::ostream& os) const
{
	SolutionMetrics::print_head(os);
	os << ",neighborhood_type";
}

void LocalSearch::Metrics::print(std::ostream& os) const
{
	SolutionMetrics::print(os);
	os << ',' << neighborhood_type;
}

LocalSearch::LocalSearch(Instance* instance)
	: instance_(instance), initial_solution_(instance, true), current_solution_(initial_solution_)
{
	improvement_strategy_ = FIRST_IMPROVEMENT;
	neighborhood_type_ = MOVE_REQUEST_WITHIN_ROUTE;

	if (std::filesystem::exists("solutions/current_solution.sln"))
		initial_solution_ = Solution::load_from_file("solutions/current_solution.sln", instance);
	reset();
}

size_t LocalSearch::step()
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
	metrics.neighborhood_type = int(neighborhood_type_);

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
			if (next_cost >= current_cost_)
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

void LocalSearch::set_initial_solution(const Solution& soln)
{
	initial_solution_ = soln;
}

Solution LocalSearch::get_current_solution() { return current_solution_; }

void LocalSearch::draw()
{
	static const char* neighborhood_labels[]{"Move Request To Another Route",
		"Swap Requests Between Routes", "Move Request Within Route"};
	ImGui::Combo(
		"Neighborhood", reinterpret_cast<int*>(&neighborhood_type_), neighborhood_labels, 3);

	static const char* strat_labels[]{"First", "Best"};
	ImGui::Combo(
		"Improvement Strat", reinterpret_cast<int*>(&improvement_strategy_), strat_labels, 2);

	if (ImGui::Button("Load Solution from File"))
	{
		if (std::filesystem::exists("solutions/current_solution.sln"))
			initial_solution_ =
				Solution::load_from_file("solutions/current_solution.sln", instance_);
	}
}
