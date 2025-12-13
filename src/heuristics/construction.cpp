#include "construction.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <ostream>
#include <random>
#include <ranges>
#include <set>
#include <unordered_set>

#include "core/solution.h"
#include "imgui.h"


void GreedyConstruction::Metrics::draw()
{
	SolutionMetrics::draw();
	ImGui::Text("Accepted Requests: %zu", accepted_requests);
	ImGui::Text("Served Requests: %zu", served_requests);
	for (size_t i = 0; i < driver_loads.size(); ++i)
		ImGui::Text("Driver %zu Load: %d", i, driver_loads[i]);
}

void GreedyConstruction::Metrics::print_head(std::ostream& os) const
{
	SolutionMetrics::print_head(os);
	os << ",accepted,served";
}

void GreedyConstruction::Metrics::print(std::ostream& os) const
{
	SolutionMetrics::print(os);
	os << "," << accepted_requests << "," << served_requests;
}

void GreedyConstruction::reset()
{
	Solution empty_solution(instance);
	state.step = 0;
	state.done = false;
	state.accepted_requests = 0;
	state.served_requests = 0;
	state.busy_drivers.clear();
	state.finished_drivers.clear();
	state.unvisited_pickups.clear();
}


GreedyConstruction::GreedyConstruction(Instance* instance)
	: instance(instance)
{
	reset();
}

Solution GreedyConstruction::get_current_solution()
{
	Solution sol(instance, true);
	metrics.driver_loads.resize(instance->vehicle_count);
	metrics.step = state.step;
	metrics.accepted_requests = state.accepted_requests;
	metrics.served_requests = state.served_requests;
	for (const auto& [time, driver] : state.busy_drivers)
	{
		metrics.driver_loads[driver.id] = driver.load;
		Route& route = sol.routes.at(driver.id);
		route.nodes = {driver.path.begin(), driver.path.end()};
		route.total_distance = sol.calculate_route_time(route);
	}
	for (auto& driver : state.finished_drivers)
	{
		metrics.driver_loads[driver.id] = driver.load;
		Route& route = sol.routes.at(driver.id);
		route.nodes = {driver.path.begin(), driver.path.end()};
		route.total_distance = sol.calculate_route_time(route);
	}
	sol.calculate_metrics(metrics);
	return sol;
}

bool GreedyConstruction::done()
{
	return state.done;
}

size_t RandomConstruction::step()
{
	static thread_local std::mt19937 rng(std::random_device{}());
	State& s = state;
	if (s.done)
		return s.step;
	if (s.step == 0)
	{
		for (Node& pickup : instance->pickups)
			s.unvisited_pickups.insert(&pickup);
		for (size_t i = 0; i < instance->vehicle_count; ++i)
			s.busy_drivers.insert({0, Driver(i)});
	}

	// select the driver that is furthest behind
	Driver driver = std::move(s.busy_drivers.begin()->second);
	s.busy_drivers.erase(s.busy_drivers.begin());

	const bool pickup_possible = !s.unvisited_pickups.empty()
		&& s.accepted_requests < instance->min_requests
		&& driver.load < instance->vehicle_capacity;
	const bool dropoff_possible = !driver.loaded_requests.empty();

	if (!pickup_possible && !dropoff_possible)
	{
		s.finished_drivers.emplace_back(std::move(driver));
		if (s.busy_drivers.empty())
			s.done = true;
		return s.step;
	}
	else if (!dropoff_possible || (pickup_possible && rng() % 2 == 0))
	{
		// must pickup
		std::vector<Node*> random_pickups(s.unvisited_pickups.begin(), s.unvisited_pickups.end());
		std::ranges::shuffle(random_pickups, rng);
		for (Node* pickup : random_pickups)
		{
			if (!driver.fits(pickup, *instance))
				continue;
			driver.visit(pickup, *instance);
			s.unvisited_pickups.erase(pickup);
			s.accepted_requests++;
			s.busy_drivers.insert({driver.time, std::move(driver)});
			return ++s.step;
		}
	}

	// did not pick anything up, so dropoff
	auto it = driver.loaded_requests.begin();
	std::advance(it, rng() % driver.loaded_requests.size());
	CustomerRequest* request = *it;
	driver.visit(request->dropoff_node, *instance);
	s.served_requests++;
	s.busy_drivers.insert({driver.time, std::move(driver)});
	return ++s.step;
}


size_t GreedyNearestAny::step()
{
	GreedyConstruction::State& s = state;
	if (s.done)
		return s.step;

	if (s.step == 0)
	{
		for (Node& pickup : instance->pickups)
			s.unvisited_pickups.insert(&pickup);
		// find the closest pickup locations to the depot
		std::vector<Node*> pickups_near_depot = {s.unvisited_pickups.begin(), s.unvisited_pickups.end()};
		std::ranges::stable_sort(pickups_near_depot, [&](Node* a, Node* b)
			{ return instance->get_distance(a) < instance->get_distance(b); }
		);
		for (size_t i = 0; i < std::min<size_t>(instance->vehicle_count, pickups_near_depot.size()); ++i)
		{
			Node* pickup = pickups_near_depot[i]; // next closest pickup
			s.unvisited_pickups.erase(pickup); // remove from unvisited nodes
			Driver driver{i};
			driver.visit(pickup, *instance);
			s.busy_drivers.insert({driver.time, driver});
			s.accepted_requests++;
		}
	}
	else if (s.busy_drivers.empty())
	{
		s.done = true;
		return s.step;
	}

	// select the driver that is furthest behind
	Driver driver = std::move(s.busy_drivers.begin()->second);
	s.busy_drivers.erase(s.busy_drivers.begin());

	// find the closest node that is either...
	std::pair<dist_t, Node*> closest = {UINT16_MAX, nullptr};
	// ...a dropoff for a loaded request...
	for (CustomerRequest* request : driver.loaded_requests)
	{
		const auto dist = instance->get_distance(request->dropoff_node, driver.path.back());
		if (dist < closest.first)
			closest = {dist, request->dropoff_node};
	}
	// ...or a pickup of a new one that fits (only if we still need more requests)
	if (s.accepted_requests < instance->min_requests)
	{
		for (Node* pickup : s.unvisited_pickups)
		{
			if (driver.load + pickup->capacity_change > instance->vehicle_capacity)
				continue; // would exceed capacity
			const auto dist = instance->get_distance(pickup, driver.path.back());
			if (dist < closest.first)
				closest = {dist, pickup};
		}
	}

	Node* closest_node = closest.second;

	// no more suitible nodes, driver is finished
	if (closest_node == nullptr)
	{
		s.finished_drivers.emplace_back(std::move(driver));
		return s.step;
	}

	// add the node to the driver's path
	driver.visit(closest_node, *instance);
	s.busy_drivers.insert({driver.time, std::move(driver)});

	if (closest_node->is_pickup())
	{
		s.unvisited_pickups.erase(closest_node);
		s.accepted_requests++;
	}
	else
		s.served_requests++;
	return ++s.step;
}

size_t GreedyRandomizedClosest::step()
{
		GreedyConstruction::State& s = state;
	if (s.done)
		return s.step;

	if (s.step == 0)
	{
		for (Node& pickup : instance->pickups)
			s.unvisited_pickups.insert(&pickup);
		// find the closest pickup locations to the depot
		std::vector<Node*> pickups_near_depot = {s.unvisited_pickups.begin(), s.unvisited_pickups.end()};
		std::ranges::stable_sort(pickups_near_depot, [&](Node* a, Node* b)
			{ return instance->get_distance(a) < instance->get_distance(b); }
		);
		for (size_t i = 0; i < std::min<size_t>(instance->vehicle_count, pickups_near_depot.size()); ++i)
		{
			Node* pickup = pickups_near_depot[i]; // next closest pickup
			s.unvisited_pickups.erase(pickup); // remove from unvisited nodes
			Driver driver{i};
			driver.visit(pickup, *instance);
			s.busy_drivers.insert({driver.time, driver});
			s.accepted_requests++;
		}
	}
	else if (s.busy_drivers.empty())
	{
		s.done = true;
		return s.step;
	}

	// select the driver that is furthest behind
	Driver driver = std::move(s.busy_drivers.begin()->second);
	s.busy_drivers.erase(s.busy_drivers.begin());

	// find the closest node that is either...
	std::map<dist_t, Node*> closest;
	const size_t reduced_candidates_count =
		std::max(1, candidates_count - (int(s.step) / tightening_interval));
	auto add_candidate = [&](float cost, Node* node)
	{
		if (closest.size() < reduced_candidates_count)
		{
			closest.insert({cost, std::move(node)});
		}
		else if (cost < closest.rbegin()->first)
		{
			closest.erase(std::prev(closest.end()));
			closest.insert({cost, std::move(node)});
		}
	};
	// ...a dropoff for a loaded request...
	for (CustomerRequest* request : driver.loaded_requests)
	{
		const auto dist = instance->get_distance(request->dropoff_node, driver.path.back());
		add_candidate(dist, request->dropoff_node);
	}
	// ...or a pickup of a new one that fits (only if we still need more requests)
	if (s.accepted_requests < instance->min_requests)
	{
		for (Node* pickup : s.unvisited_pickups)
		{
			if (driver.load + pickup->capacity_change > instance->vehicle_capacity)
				continue; // would exceed capacity
			add_candidate(instance->get_distance(pickup, driver.path.back()), pickup);
		}
	}

	// no more suitible nodes, driver is finished
	if (closest.empty())
	{
		s.finished_drivers.emplace_back(std::move(driver));
		return s.step;
	}
	// randomly select one of the candidates
	Node* closest_node = nullptr;
	size_t index = std::rand() % closest.size();
	auto it = closest.begin();;
	std::advance(it, index);
	closest_node = it->second;

	// add the node to the driver's path
	driver.visit(closest_node, *instance);
	s.busy_drivers.insert({driver.time, std::move(driver)});

	if (closest_node->is_pickup())
	{
		s.unvisited_pickups.erase(closest_node);
		s.accepted_requests++;
	}
	else
		s.served_requests++;
	return ++s.step;
}

void GreedyRandomizedClosest::draw()
{
	ImGui::SliderInt("Candidates Count", &candidates_count, 1, 5);
	ImGui::SliderInt("Tightening Interval", &tightening_interval, 1, 50);
}


GreedyNearestWeighted::GreedyNearestWeighted(Instance* instance)
	: GreedyConstruction(instance)
{
	int edge_count = instance->node_count * (instance->node_count - 1) / 2;
	uint64_t total_distance = 0;
	for (size_t i = 0; i < instance->node_count; ++i)
	{
		for (size_t j = i + 1; j < instance->node_count; ++j)
		{
			total_distance += instance->get_distance(&instance->nodes[i], &instance->nodes[j]);
		}
	}
	mean_distance_ = total_distance / edge_count;
}

size_t GreedyNearestWeighted::step()
{
	GreedyConstruction::State& s = state;
	if (s.done)
		return s.step;

	if (s.step == 0)
	{
		for (Node& pickup : instance->pickups)
			s.unvisited_pickups.insert(&pickup);
		// find the closest pickup locations to the depot
		std::vector<Node*> pickups_near_depot = {s.unvisited_pickups.begin(), s.unvisited_pickups.end()};
		std::ranges::stable_sort(pickups_near_depot, [&](Node* a, Node* b)
			{ return instance->get_distance(a) < instance->get_distance(b); }
		);
		for (size_t i = 0; i < std::min<size_t>(instance->vehicle_count, pickups_near_depot.size()); ++i)
		{
			Node* pickup = pickups_near_depot[i]; // next closest pickup
			s.unvisited_pickups.erase(pickup); // remove from unvisited nodes
			Driver driver{i};
			driver.visit(pickup, *instance);
			s.busy_drivers.insert({driver.time, driver});
		}
	}
	else if (s.busy_drivers.empty())
	{
		s.done = true;
		return s.step;
	}

	// select the driver that is furthest behind
	Driver driver = std::move(s.busy_drivers.begin()->second);
	s.busy_drivers.erase(s.busy_drivers.begin());
	
	std::pair<dist_t, Node*> closest_pickup = {UINT16_MAX, nullptr};
	if (s.accepted_requests < instance->min_requests)
	{
		for (Node* pickup : s.unvisited_pickups)
		{
			if (!driver.fits(pickup, *instance))
				continue; // would exceed capacity
			const auto dist = instance->get_distance(pickup, driver.path.back());
			if (dist < closest_pickup.first)
				closest_pickup = {dist, pickup};
		}
	}
	std::pair<dist_t, Node*> closest_dropoff = {UINT16_MAX, nullptr};
	for (CustomerRequest* request : driver.loaded_requests)
	{
		const auto dist = instance->get_distance(request->dropoff_node, driver.path.back());
		if (dist < closest_dropoff.first)
			closest_dropoff = {dist, request->dropoff_node};
	}

	Node* selected_node = nullptr;
	if (closest_pickup.second == nullptr && closest_dropoff.second == nullptr)
	{
		// no more suitible nodes, driver is finished
		s.finished_drivers.emplace_back(std::move(driver));
		return s.step;
	}
	else if (closest_pickup.second == nullptr)
		selected_node = closest_dropoff.second;
	else if (closest_dropoff.second == nullptr)
		selected_node = closest_pickup.second;
	else
	{
		// both available, choose based on load factor
		const float pickup_weight = 1.f - (closest_pickup.first / (float) mean_distance_);
		const float dropoff_weight = 1.f - (closest_dropoff.first / (float) mean_distance_);
		const float load_factor = driver.load / (float) instance->vehicle_capacity;
		const float pickup_score = pickup_weight * (1.f - load_factor);
		const float dropoff_score = dropoff_weight * load_factor * dropoff_bias_;
		selected_node = (pickup_score >= dropoff_score)
			? closest_pickup.second
			: closest_dropoff.second;
	}

	// add the node to the driver's path
	std::cout << "Driver " << driver.id << " visits "
		<< (selected_node->is_pickup() ? "pickup" : "dropoff") << " -> load: " << driver.load << "\n";
	driver.visit(selected_node, *instance);
	s.busy_drivers.insert({driver.time, std::move(driver)});

	if (selected_node->is_pickup())
	{
		s.unvisited_pickups.erase(selected_node);
		s.accepted_requests++;
	}
	else
		s.served_requests++;
	return ++s.step;
}

void GreedyNearestWeighted::draw()
{
	ImGui::Text("Mean Distance: %f", mean_distance_);
	ImGui::SliderFloat("Dropoff Bias", &dropoff_bias_, .2f, 20.f);
}

