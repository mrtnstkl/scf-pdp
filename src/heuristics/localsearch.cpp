#include "heuristics/localsearch.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include "neighborhoods.h"


LocalSearch::LocalSearch(Instance* instance)
	: instance(instance), current_solution_(instance)
{
	reset();
}

void LocalSearch::reset()
{
	// Start from an empty solution (or however you want)
	current_solution_ = Solution(instance);

	// Compute metrics & cost of starting solution
	current_solution_.calculate_metrics(metrics);
	metrics.step = 0;
	finished = false;

	// keep these in sync: current_cost_ reflects the *objective*
	current_cost_ = metrics.total_cost;
}


size_t LocalSearch::step()
{
	if (finished)
		return metrics.step;

	if (!neighborhood_ || !neighborhood_->has_next())
	{
		// No more neighbours to explore: local optimum for this neighbourhood
		finished = true;
		return metrics.step;
	}

	// Ask neighbourhood for ONE candidate neighbour
	float candidate_cost = neighborhood_->next();
	Solution candidate_solution = neighborhood_->get();

	// First-improvement: accept immediately if strictly better
	if (candidate_cost < current_cost_)
	{
		// Accept
		current_solution_ = std::move(candidate_solution);
		current_cost_ = candidate_cost;

		// Recompute metrics for GUI
		current_solution_.calculate_metrics(metrics);

		// Restart neighbourhood from the new current solution
		switch (selected_neighborhood_)
		{
		case NeighborhoodType::MoveWithinRoute:
			neighborhood_ =
				std::make_unique<MoveRequestWithinRouteNeighborhood>(current_solution_, instance);
			break;
		//case NeighborhoodType::SwapBetweenVehicles:
		//    neighborhood_ = std::make_unique<SwapRequestsBetweenVehiclesNeighborhood>(
		//        current_solution_, instance);
		//    break;
		case NeighborhoodType::MoveBetweenVehicles:
			neighborhood_ = std::make_unique<MoveRequestBetweenVehiclesNeighborhood>(
				current_solution_, instance);
			break;
		}
	}
	else
	{
		// Reject: keep current_solution_ and current_cost_ unchanged
		// just move on to next neighbour on the next call
	}

	++metrics.step; // count neighbour evaluations

	return metrics.step;
}


bool LocalSearch::done() { return finished; }

Solution LocalSearch::get_current_solution() { return current_solution_; }

void LocalSearch::load_solution_from_file(const std::string& path)
{
	std::ifstream in(path);
	if (!in)
	{
		std::cerr << "Failed to open solution file: " << path << std::endl;
		return;
	}

	std::string header;
	std::getline(in, header);
	if (header != "solution")
	{
		std::cerr << "Invalid solution header in " << path << std::endl;
		return;
	}

	current_solution_ = Solution(instance);

	std::string line;
	while (std::getline(in, line))
	{
		if (line.empty())
			continue;

		std::istringstream ss(line);
		Route route;
		route.depot = instance->depot;

		int idx1based;
		while (ss >> idx1based)
		{
			int idx0 = idx1based - 1;
			if (idx0 < 0 || idx0 >= (int)instance->node_count)
			{
				std::cerr << "Invalid node index " << idx1based << " in solution file."
						  << std::endl;
				continue;
			}
			Node* node = &instance->nodes[idx0];
			route.nodes.push_back(node);
		}

		route.total_distance = current_solution_.calculate_route_time(route);
		current_solution_.routes.push_back(std::move(route));
	}

    current_solution_.calculate_metrics(metrics);
    metrics.step = 0;
    finished = false;

    // Keep current_cost_ in sync with metrics
    current_cost_ = metrics.total_cost;
}

void LocalSearch::draw()
{
	// Neighbourhood selection UI
	const char* names[] = {"Move Within Route",
		//"Swap Between Vehicles",
		"Move Between Vehicles"};
	int current = static_cast<int>(selected_neighborhood_);
	if (ImGui::Combo("Neighborhood", &current, names, IM_ARRAYSIZE(names)))
	{
		selected_neighborhood_ = static_cast<NeighborhoodType>(current);
		reset();
	}

	if (ImGui::Button("Load current solution"))
	{
		load_solution_from_file("solutions/current_solution.sln");
	}

	ImGui::Text("Step: %zu", metrics.step);
	ImGui::Text("Total Cost: %.2f", metrics.total_cost);
	ImGui::Text("Fairness: %.4f", metrics.fairness);
}
