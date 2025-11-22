#include "solution.h"

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <unordered_set>


Solution::Solution(Instance* instance, bool init)
	: instance(instance)
{
	if (!init)
		return;
	routes.resize(instance->vehicle_count);
	for (auto& route : routes)
		route.depot = instance->depot;
}

uint64_t Solution::calculate_route_time(const Route& route) const
{
	if (route.nodes.empty())
		return 0;

	uint64_t total = instance->get_distance(route.nodes.front()) + instance->get_distance(route.nodes.back());
	if (route.nodes.size() == 1)
		return total;

	for (auto it = route.nodes.begin(); it != std::prev(route.nodes.end()); ++it)
	{
		total += instance->get_distance(*it, *std::next(it));
	}
	return total;
}

float Solution::calculate_jain_fairness() const
{
	const int n = instance->request_count;
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
	int route_sum = 0;
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

bool Solution::validate() const
{
	if (routes.size() != instance->vehicle_count)
		return false; // unexpected route count

	std::unordered_set<CustomerRequest*> all_served;
	for (const Route& route : routes)
	{
		std::unordered_set<CustomerRequest*> current_requests;
		int current_load = 0;
		for (const Node* node : route.nodes)
		{
			CustomerRequest* cr = &instance->get_request_from_node(node);
			if (node->is_pickup())
			{
				if (all_served.contains(cr))
					return false; // already served
				current_requests.insert(cr);
				all_served.insert(cr);
			}
			else
			{
				if (!current_requests.contains(cr))
					return false; // dropoff without pickup
				current_requests.erase(cr);
			}
			current_load += node->capacity_change;
			if (current_load < 0 || current_load > instance->vehicle_capacity)
				return false; // capacity violation
		}
	}
	if (all_served.size() < instance->min_requests)
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

void Solution::draw(Canvas& canvas) const
{
	instance->draw(canvas, true);
	Color::random(123);
	for (const auto& route : routes)
	{
		Color c = Color::random();
		if (route.nodes.empty()) continue;
		canvas.add(Line{ route.depot.x, route.depot.y, route.nodes.front()->location.x, route.nodes.front()->location.y, c });
		for (auto it = route.nodes.begin(); it != std::prev(route.nodes.end()); ++it)
		{
			auto& current = *it;
			auto& next = *std::next(it);
			canvas.add(Line{ current->location.x, current->location.y, next->location.x, next->location.y, c });
		}
		canvas.add(Line{ route.nodes.back()->location.x, route.nodes.back()->location.y, route.depot.x, route.depot.y, c });
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
