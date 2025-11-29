#include "localsearch.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <unordered_set>
#include "core/solution.h"

enum Neighbourhood
{
	RELOCATE,
	SWAP,
	TWO_OPT,
	COUNT
};


LocalSearch::LocalSearch(Instance* instance)
	: instance(instance), current_solution_(instance)
{
	reset();
}

void LocalSearch::reset()
{
	metrics = Metrics();
	current_solution_ = Solution(instance);
}

size_t LocalSearch::step()
{
	if (k_ >= static_cast<size_t>(Neighbourhood::COUNT))
		k_ = 0;

	Solution candidate = current_solution_;
	bool success = false;

	switch (static_cast<Neighbourhood>(k_))
	{
	case Neighbourhood::RELOCATE:
		success = apply_relocate(candidate);
		break;

	case Neighbourhood::SWAP:
		success = apply_swap(candidate);
		break;

	case Neighbourhood::TWO_OPT:
		success = apply_two_opt(candidate);
		break;

	default:
		break;
	}

	if (success && is_solution_feasible(candidate)
		&& candidate.calculate_cost() < current_solution_.calculate_cost())
	{
		current_solution_ = std::move(candidate);
		k_ = 0; // restart VND
		improved_ = true;
	}
	else
	{
		++k_; // next neighbourhood
		improved_ = false;
	}

	current_solution_.calculate_metrics(metrics);
	++metrics.step;

	return metrics.step;
}

void LocalSearch::draw()
{
	if (ImGui::Button("Load current solution"))
	{
		load_solution_from_file("solutions/current_solution.sln");
	}

	if (ImGui::Button("VND step"))
	{
		step();
	}
}

bool LocalSearch::apply_relocate(Solution& sol)
{
	// Works with containers that only provide bidirectional iterators (e.g., std::list)
	for (auto& route : sol.routes)
	{
		// iterate by index but use std::next to obtain iterators
		for (size_t i = 0; i < route.nodes.size(); ++i)
		{
			auto it_i = std::next(route.nodes.begin(),
				static_cast<
					typename std::iterator_traits<decltype(route.nodes.begin())>::difference_type>(
					i));
			Node* moved = *it_i;

			// remove node
			route.nodes.erase(it_i);

			for (auto& target_route : sol.routes)
			{
				for (size_t j = 0; j <= target_route.nodes.size(); ++j)
				{
					auto insert_pos = std::next(target_route.nodes.begin(),
						static_cast<typename std::iterator_traits<
							decltype(target_route.nodes.begin())>::difference_type>(j));
					target_route.nodes.insert(insert_pos, moved);

					if (is_route_feasible(route) && is_route_feasible(target_route))
					{
						auto new_cost = sol.calculate_route_time(target_route);

						if (new_cost + route.total_distance < sol.calculate_cost())
						{
							route.total_distance = sol.calculate_route_time(route);
							target_route.total_distance = new_cost;
							sol.calculate_cost();
							return true;
						}
					}


					// undo insert
					auto erase_pos = std::next(target_route.nodes.begin(),
						static_cast<typename std::iterator_traits<
							decltype(target_route.nodes.begin())>::difference_type>(j));
					target_route.nodes.erase(erase_pos);
				}
			}

			// undo erase: insert back at original position
			auto insert_back = std::next(route.nodes.begin(),
				static_cast<
					typename std::iterator_traits<decltype(route.nodes.begin())>::difference_type>(
					i));
			route.nodes.insert(insert_back, moved);
		}
	}

	return false;
}

bool LocalSearch::apply_swap(Solution& sol)
{
	// Use iterator-aware swaps so this works with non-random-access containers
	for (auto& r1 : sol.routes)
	{
		for (auto& r2 : sol.routes)
		{
			for (size_t i = 0; i < r1.nodes.size(); ++i)
			{
				auto it1 = std::next(r1.nodes.begin(),
					static_cast<
						typename std::iterator_traits<decltype(r1.nodes.begin())>::difference_type>(
						i));
				for (size_t j = 0; j < r2.nodes.size(); ++j)
				{
					auto it2 = std::next(r2.nodes.begin(),
						static_cast<typename std::iterator_traits<
							decltype(r2.nodes.begin())>::difference_type>(j));
					std::iter_swap(it1, it2);

					if (is_route_feasible(r1) && is_route_feasible(r2))
					{
						auto new1 = sol.calculate_route_time(r1);
						auto new2 = sol.calculate_route_time(r2);

						if (new1 + new2 < r1.total_distance + r2.total_distance)
						{
							r1.total_distance = new1;
							r2.total_distance = new2;
							sol.calculate_cost();
							return true;
						}
					}


					std::iter_swap(it1, it2); // rollback
				}
			}
		}
	}
	return false;
}

bool LocalSearch::apply_two_opt(Solution& sol)
{
	for (auto& route : sol.routes)
	{
		for (size_t i = 0; i + 1 < route.nodes.size(); ++i)
		{
			for (size_t j = i + 1; j < route.nodes.size(); ++j)
			{
				auto first = std::next(route.nodes.begin(),
					static_cast<typename std::iterator_traits<
						decltype(route.nodes.begin())>::difference_type>(i));
				auto last = std::next(route.nodes.begin(),
					static_cast<typename std::iterator_traits<
						decltype(route.nodes.begin())>::difference_type>(j + 1));

				std::reverse(first, last);

				auto new_cost = sol.calculate_route_time(route);

				if (is_route_feasible(route) && new_cost < route.total_distance)
				{
					route.total_distance = new_cost;
					sol.calculate_cost();
					return true;
				}

				std::reverse(first, last);
			}
		}
	}
	return false;
}

bool LocalSearch::is_route_feasible(const Route& route) const
{
	int load = 0;

	std::unordered_set<int> seen_requests;

	for (Node* node : route.nodes)
	{
		const auto& req = instance->get_request_from_node(node);

		load += node->capacity_change;

		if (load < 0)
			return false;

		if (load > instance->vehicle_capacity)
			return false;

		// pickup
		if (node->capacity_change > 0)
		{
			seen_requests.insert(req.id);
		}
		else // dropoff
		{
			if (!seen_requests.contains(req.id))
				return false; // drop before pickup
		}
	}

	return true;
}

bool LocalSearch::is_solution_feasible(const Solution& sol) const
{
	for (const auto& r : sol.routes)
	{
		if (!is_route_feasible(r))
			return false;
	}
	return true;
}


Solution LocalSearch::get_current_solution() { return current_solution_; }

bool LocalSearch::done() { return k_ >= static_cast<size_t>(Neighbourhood::COUNT); }

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

	// Build a fresh solution
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
			int idx0based = idx1based - 1;

			// Validate
			if (idx0based < 0 || idx0based >= (int)instance->node_count)
			{
				std::cerr << "Invalid node index " << idx1based << " in solution file (valid: 1–"
						  << instance->node_count << ")." << std::endl;
				continue;
			}

			// Retrieve node
			Node* node = &instance->nodes[idx0based];
			route.nodes.push_back(node);
		}

		// Compute route distance and store
		route.total_distance = current_solution_.calculate_route_time(route);
		current_solution_.routes.push_back(std::move(route));
	}

	current_solution_.calculate_metrics(metrics);
	metrics.step++;
}