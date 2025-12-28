#include "solution.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <numeric>

#include "core/instance.h"

#ifndef SCFPDP_HEADLESS
#include "imgui.h"
#endif // SCFPDP_HEADLESS

void SolutionMetrics::draw()
{
#ifndef SCFPDP_HEADLESS
	ImGui::Text("Step: %zu", step);
    ImGui::Text("Route Count: %zu", route_count);
	ImGui::Text("Total Cost: %.2f", total_cost);
	ImGui::Text("Fairness: %.4f", fairness);
	ImGui::Text("Valid: %s", valid ? "Yes" : "No");
#endif // SCFPDP_HEADLESS
}

void SolutionMetrics::print_head(std::ostream& os) const
{
    os << "step,route_count,total_cost,fairness,valid";
}

void SolutionMetrics::print(std::ostream& os) const
{
	os
        << step << ','
        << route_count << ','
		<< total_cost << ','
        << fairness << ','
        << valid;
}

std::ostream& operator<<(std::ostream& os, const SolutionMetrics& metrics)
{
    metrics.print(os);
    return os;
}

Solution::Solution(Instance* instance, bool init)
	: instance(instance)
{
	if (!init)
		return;
	routes.resize(instance->vehicle_count);
	for (auto& route : routes)
		route.depot = instance->depot;
}

float Solution::calculate_jain_fairness() const
{
	const uint64_t n = instance->vehicle_count;
	uint64_t route_sum = 0;
	uint64_t route_sq_sum = 0;
	for (const auto& route : routes)
	{
		const uint64_t d = route.total_distance;
		route_sum += d;
		route_sq_sum += d * d;
	}
	return route_sum * route_sum / float(n * route_sq_sum);
}

float Solution::calculate_cost() const
{
	uint64_t route_sum = 0;
	for (const auto& route : routes)
		route_sum += route.total_distance;
	return route_sum + instance->fairness * (1.f - calculate_jain_fairness());
}

void Solution::calculate_metrics(SolutionMetrics& metrics) const
{
	metrics.route_count = routes.size();
	metrics.total_cost = calculate_cost();
	metrics.fairness = calculate_jain_fairness();
	metrics.valid = validate();
}

bool Solution::validate(bool only_structure) const
{
	if (routes.size() != instance->vehicle_count)
		return false; // unexpected route count

	std::unordered_set<const Node*> visited_nodes;
	std::unordered_map<CustomerRequest*, size_t> open_requests;

	for (size_t r = 0; r < routes.size() ; ++r)
	{
		int current_load = 0;
		for (const Node* node : routes[r].nodes)
		{
			if (!visited_nodes.insert(node).second)
				return false; // node already visited

			CustomerRequest* cr = &instance->get_request_from_node(node);
			if (node->is_pickup())
			{
				open_requests.insert({cr, r});
			}
			else
			{
				const auto cr_it = open_requests.find(cr);
				if (cr_it == open_requests.end())
					return false; // dropoff without pickup
				if (cr_it->second != r)
					return false; // dropoff in different route than pickup
				open_requests.erase(cr_it);
			}
			current_load += node->capacity_change;
			if (current_load < 0 || current_load > instance->vehicle_capacity)
				return false; // capacity violation
		}
		if (!open_requests.empty())
			return false; // not all pickups were dropped off
	}
	if (!only_structure && visited_nodes.size() / 2 < instance->min_requests)
		return false; // less customer requests have been fullfilled than required
	return true;
}

void Solution::print() const
{
	std::cout << "Solution with cost: " << calculate_cost() << "\n";
	if (!validate())
	{
		std::cout << "Solution is INVALID!!!\n";
		return;
	}
	for (size_t i = 0; i < routes.size(); ++i)
	{
		const Route& route = routes[i];
		std::cout << "Route " << i << "): Depot -> ";
		for (const Node* node : route.nodes)
		{
			std::cout << (node->is_pickup() ? "P" : "D") << instance->get_index(node) << " -> ";
		}
		std::cout << "Depot\n";
	}
}

void Solution::visualize(Canvas& canvas) const
{
	instance->draw(canvas, true);
	Color::random(123);
	for (const auto& route : routes)
	{
		Color c = Color::random();
		if (route.nodes.empty()) continue;
		canvas.add(Line{ 
			float(route.depot.x), float(route.depot.y), 
			float(route.nodes.front()->location.x), float(route.nodes.front()->location.y), c });
		for (auto it = route.nodes.begin(); it != std::prev(route.nodes.end()); ++it)
		{
			auto& current = *it;
			auto& next = *std::next(it);
			canvas.add(
				Line{ float(current->location.x), float(current->location.y), 
				float(next->location.x), float(next->location.y), c });
		}
		canvas.add(
			Line{ float(route.nodes.back()->location.x), float(route.nodes.back()->location.y), 
				float(route.depot.x), float(route.depot.y), c });
	}
}

void Solution::write_to_file(const char* filename, const char* comment) const
{
	std::ofstream file(filename);
	if (!file.is_open())
		throw std::runtime_error("Could not open solution file for writing");
	file << comment << std::endl;
	for (const auto& route : routes)
	{
		for (const auto node : route.nodes)
			file << instance->get_index(node) + 1 << " ";
		file << std::endl;
	}
}

Solution Solution::load_from_file(const char* path, Instance* instance)
{
	std::ifstream in(path);
	if (!in)
	{
		std::cerr << "Failed to open solution file: " << path << std::endl;
		throw std::runtime_error("Could not open solution file for reading");
	}

	std::string header;
	std::getline(in, header);

	// Build a fresh solution
	Solution sol(instance);

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

			// validate
			if (idx0based < 0 || idx0based >= (int)instance->node_count)
			{
				std::cerr << "Invalid node index " << idx1based << " in solution file (valid: 1–"
						  << instance->node_count << ")." << std::endl;
				continue;
			}

			// retrieve node
			Node* node = &instance->nodes[idx0based];
			route.nodes.push_back(node);
		}
		// compute route distance and store
		route.total_distance = instance->calculate_route_time(route);
		sol.routes.push_back(std::move(route));
	}
	return sol;
 }


DeltaCalculator::DeltaCalculator(Instance* instance)
	: instance(instance), route_sum(0), route_squares(instance->vehicle_count, 0)
{
}

DeltaCalculator::DeltaCalculator(Instance* instance, const Solution& solution)
	: DeltaCalculator(instance)
{
	route_sum = 0;
	for (size_t i = 0; i < solution.routes.size(); ++i)
	{
		const Route& route = solution.routes[i];
		const auto d = instance->calculate_route_time(route);
		route_sum += d;
		route_squares[i] = d * d;
	}
}

DeltaCalculator::DeltaCalculator(const Solution& solution)
	: DeltaCalculator(solution.instance)
{
	route_sum = 0;
	for (size_t i = 0; i < solution.routes.size(); ++i)
	{
		const Route& route = solution.routes[i];
		const auto d = instance->calculate_route_time(route);
		route_sum += d;
		route_squares[i] = d * d;
	}
}

void DeltaCalculator::add_stop(int route_id, const Node* prev_node, const Node* new_node)
{
	if (prev_node == nullptr)
	{
		// this is the first stop of the route, so use depot distance (twice)
		const auto d_depot = instance->get_distance(new_node);
		route_sum += d_depot + d_depot;
		route_squares[route_id] = d_depot * d_depot + d_depot * d_depot;
		return;
	}
	// we are inserting at the end, so we:
	// 1. add our own depot distance
	const auto d_depot_new = instance->get_distance(new_node);
	route_sum += d_depot_new;
	route_squares[route_id] += d_depot_new * d_depot_new;
	// 2. subtract the old depot distance
	const auto d_depot_old = instance->get_distance(prev_node);
	route_sum -= d_depot_old;
	route_squares[route_id] -= d_depot_old * d_depot_old;
	// 3. add the distance to the previous node
	const auto d_prev = instance->get_distance(prev_node, new_node);
	route_sum += d_prev;
	route_squares[route_id] += d_prev * d_prev;
}

void DeltaCalculator::add_stop(int route_id, const Node* prev_node, const Node* new_node, const Node* next_node)
{
	if (next_node == nullptr)
	{
		add_stop(route_id, prev_node, new_node);
		return;
	}
	if (prev_node == nullptr)
	{
		// we are adding something at the beginning
		// this is equivalent to adding to the end, we just use next in place of prev
		add_stop(route_id, next_node, new_node);
		return;
	}
	// we are inserting between two nodes, so we:
	// 1. add the distance to the first
	const auto d_prev = instance->get_distance(new_node, prev_node);
	route_sum += d_prev;
	route_squares[route_id] += d_prev * d_prev;
	// 2. add the distance to the second
	const auto d_next = instance->get_distance(new_node, next_node);
	route_sum += d_next;
	route_squares[route_id] += d_next * d_next;
	// 3. remove the distance between the two
	const auto d_prev_next = instance->get_distance(prev_node, next_node);
	route_sum -= d_prev_next;
	route_squares[route_id] -= d_prev_next * d_prev_next;
}

void DeltaCalculator::remove_stop(int route_id, const Node* prev_node, const Node* old_node)
{
	if (prev_node == nullptr)
	{
		// we are removing the only node in the route
		route_sum = 0;
		route_squares[route_id] = 0;
		return;
	}
	// we are removing the last node in the route, so we:
	// 1. remove our own depot distance
	const auto d_depot_old = instance->get_distance(old_node);
	route_sum -= d_depot_old;
	route_squares[route_id] -= d_depot_old * d_depot_old;
	// 2. add the old depot distance
	const auto d_depot_new = instance->get_distance(prev_node);
	route_sum += d_depot_new;
	route_squares[route_id] += d_depot_new * d_depot_new;
	// 3. remove the distance to the previous node
	const auto d_prev = instance->get_distance(prev_node, old_node);
	route_sum -= d_prev;
	route_squares[route_id] -= d_prev * d_prev;
}

void DeltaCalculator::remove_stop(int route_id, const Node* prev_node, const Node* old_node, const Node* next_node)
{
	if (next_node == nullptr)
	{
		remove_stop(route_id, prev_node, old_node);
		return;
	}
	if (prev_node == nullptr)
	{
		// we are removing the first node in the route
		remove_stop(route_id, next_node, old_node);
		return;
	}
	// we are removing a node between two nodes, so we:
	// 1. remove the distance to the first
	const auto d_prev = instance->get_distance(old_node, prev_node);
	route_sum -= d_prev;
	route_squares[route_id] -= d_prev * d_prev;
	// 2. remove the distance to the second
	const auto d_next = instance->get_distance(old_node, next_node);
	route_sum -= d_next;
	route_squares[route_id] -= d_next * d_next;
	// 3. add the distance between the two
	const auto d_prev_next = instance->get_distance(prev_node, next_node);
	route_sum += d_prev_next;
	route_squares[route_id] += d_prev_next * d_prev_next;
}

void DeltaCalculator::swap_stops(int route_id, const Node* node_prev_a, const Node* node_a,
	const Node* node_b, const Node* node_after_b)
{
	// 1. remove edges between A and its predecessor
	const auto d_a_to_prev = (node_prev_a == nullptr)
		? instance->get_distance(node_a)
		: instance->get_distance(node_prev_a, node_a);
	route_sum -= d_a_to_prev;
	route_squares[route_id] -= d_a_to_prev * d_a_to_prev;
	// 2. remove edges between B and its successor
	const auto d_b_to_after = (node_after_b == nullptr)
		? instance->get_distance(node_b)
		: instance->get_distance(node_b, node_after_b);
	route_sum -= d_b_to_after;
	route_squares[route_id] -= d_b_to_after * d_b_to_after;
	// 3. add edge between A and its new successor
	const auto d_a_to_after = (node_after_b == nullptr)
		? instance->get_distance(node_a)
		: instance->get_distance(node_a, node_after_b);
	route_sum += d_a_to_after;
	route_squares[route_id] += d_a_to_after * d_a_to_after;
	// 4. add edge between B and its new predecessor
	const auto d_b_to_prev = (node_prev_a == nullptr)
		? instance->get_distance(node_b)
		: instance->get_distance(node_prev_a, node_b);
	route_sum += d_b_to_prev;
	route_squares[route_id] += d_b_to_prev * d_b_to_prev;
}


float DeltaCalculator::fairness() const
{
	return (route_sum * route_sum) / float(
		route_squares.size() *
		std::accumulate(route_squares.begin(), route_squares.end(), uint64_t(0))
	);
}

float DeltaCalculator::cost() const
{
	return route_sum + instance->fairness * (1.f - fairness());
}

float DeltaCalculator::cost(float fairness) const
{
	return route_sum + instance->fairness * (1.f - fairness);
}

std::pair<float, float> DeltaCalculator::cost_with_fairness() const
{
	const float f = fairness();
	return { cost(f), f };
}
