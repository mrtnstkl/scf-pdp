#include "construction.h"

#include <algorithm>
#include <cassert>
#include <filesystem>
#include <iostream>
#include <map>
#include <ostream>
#include <random>
#include <ranges>
#include <set>
#include <unordered_set>

#include "core/solution.h"

#ifndef SCFPDP_HEADLESS
#include "imgui.h"
#endif // SCFPDP_HEADLESS


GreedyConstruction::State::State(Solution& initial_solution)
	: step(0), done(false), accepted_requests(0), served_requests(0)
{
	for (Node& node : initial_solution.instance->pickups)
		unvisited_pickups.insert(&node);
	for (int route_id = 0; const Route& route : initial_solution.routes)
	{
		if (route.nodes.empty())
		{
			busy_drivers.insert({0, Driver(route_id++)});
			continue;
		}
		Driver driver{route_id++};
		for (Node* node : route.nodes)
		{
			driver.visit(node, *initial_solution.instance);
			if (node->is_pickup())
				unvisited_pickups.erase(node);
		}
		busy_drivers.insert({driver.time, std::move(driver)});
	}
}

void GreedyConstruction::Metrics::draw()
{
#ifndef SCFPDP_HEADLESS
	SolutionMetrics::draw();
	ImGui::Text("Accepted Requests: %zu", accepted_requests);
	ImGui::Text("Served Requests: %zu", served_requests);
	for (size_t i = 0; i < driver_loads.size(); ++i)
		ImGui::Text("Driver %zu Load: %d", i, driver_loads[i]);
#endif // SCFPDP_HEADLESS
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
	state = State(initial_solution);
}

GreedyConstruction::GreedyConstruction(Instance* instance)
	: instance(instance), initial_solution(instance, true)
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
		route.total_distance = instance->calculate_route_time(route);
	}
	for (auto& driver : state.finished_drivers)
	{
		metrics.driver_loads[driver.id] = driver.load;
		Route& route = sol.routes.at(driver.id);
		route.nodes = {driver.path.begin(), driver.path.end()};
		route.total_distance = instance->calculate_route_time(route);
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

GreedyNearestAny::GreedyNearestAny(Instance* instance)
	: GreedyConstruction(instance)
{
	reset();
}

GreedyNearestAny::GreedyNearestAny(Instance& instance, const Solution& initial_solution)
	: GreedyConstruction(&instance)
{
	this->initial_solution = initial_solution;
	reset();
}

size_t GreedyNearestAny::step()
{
	GreedyConstruction::State& s = state;
	if (s.done || s.busy_drivers.empty())
	{
		s.done = true;
		return s.step;
	}
	if (auto it = single_driver ? std::prev(s.busy_drivers.end()) : s.busy_drivers.begin(); it->first == 0)
	{
		// find the closest pickup location to the depot
		std::pair<dist_t, Node*> closest = {UINT16_MAX, nullptr};
		for (Node* pickup : s.unvisited_pickups)
		{
			const auto dist = instance->get_distance(pickup);
			if (dist < closest.first)
				closest = {dist, pickup};
		}
		Node* closest_node = closest.second;
		s.unvisited_pickups.erase(closest_node);
		it->second.visit(closest_node, *instance);
		s.busy_drivers.insert({it->second.time, std::move(it->second)});
		s.busy_drivers.erase(it);
		s.accepted_requests++;
		return ++s.step;
	}

	auto driver_it = single_driver ? std::prev(s.busy_drivers.end()) : s.busy_drivers.begin();
	Driver driver = std::move(driver_it->second);
	s.busy_drivers.erase(driver_it);

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
		if (single_driver)
		{
			for (auto& [_, d] : s.busy_drivers)
				s.finished_drivers.emplace_back(std::move(d));
			s.busy_drivers.clear();
		}
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

void GreedyNearestAny::draw()
{
#ifndef SCFPDP_HEADLESS
	ImGui::Checkbox("Single Driver Mode", &single_driver);
#endif // SCFPDP_HEADLESS
}

size_t GreedyRandomizedClosest::step()
{
	static thread_local std::mt19937 rng(std::random_device{}());

	GreedyConstruction::State& s = state;
	if (s.done || s.busy_drivers.empty())
	{
		s.done = true;
		return s.step;
	}
	if (auto it = s.busy_drivers.begin(); it->first == 0)
	{
		// find the closest pickup location to the depot
		std::pair<dist_t, Node*> closest = {UINT16_MAX, nullptr};
		for (Node* pickup : s.unvisited_pickups)
		{
			const auto dist = instance->get_distance(pickup);
			if (dist < closest.first)
				closest = {dist, pickup};
		}
		Node* closest_node = closest.second;
		s.unvisited_pickups.erase(closest_node);
		it->second.visit(closest_node, *instance);
		s.busy_drivers.insert({it->second.time, std::move(it->second)});
		s.busy_drivers.erase(it);
		s.accepted_requests++;
		return ++s.step;
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
	size_t index = rng() % closest.size();
	auto it = closest.begin();
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
#ifndef SCFPDP_HEADLESS
	ImGui::SliderInt("Candidates Count", &candidates_count, 1, 5);
	ImGui::SliderInt("Tightening Interval", &tightening_interval, 1, 50);
#endif // SCFPDP_HEADLESS
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
	if (s.done || s.busy_drivers.empty())
	{
		s.done = true;
		return s.step;
	}
	if (auto it = s.busy_drivers.begin(); it->first == 0)
	{
		// find the closest pickup location to the depot
		std::pair<dist_t, Node*> closest = {UINT16_MAX, nullptr};
		for (Node* pickup : s.unvisited_pickups)
		{
			const auto dist = instance->get_distance(pickup);
			if (dist < closest.first)
				closest = {dist, pickup};
		}
		Node* closest_node = closest.second;
		s.unvisited_pickups.erase(closest_node);
		it->second.visit(closest_node, *instance);
		s.busy_drivers.insert({it->second.time, std::move(it->second)});
		s.busy_drivers.erase(it);
		s.accepted_requests++;
		return ++s.step;
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
	//std::cout << "Driver " << driver.id << " visits "
	//	<< (selected_node->is_pickup() ? "pickup" : "dropoff") << " -> load: " << driver.load << "\n";
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
#ifndef SCFPDP_HEADLESS
	ImGui::Text("Mean Distance: %f", mean_distance_);
	ImGui::SliderFloat("Dropoff Bias", &dropoff_bias_, .2f, 20.f);
#endif // SCFPDP_HEADLESS
}

bool GreedyInsertion::State::NodeOrder::operator()(const CustomerRequest* a, const CustomerRequest* b) const
{
	// order by decreasing capacity change
	// this causes larger requests to be served first
	if (a->pickup_node->capacity_change == b->pickup_node->capacity_change)
		return a->id < b->id;
	return a->pickup_node->capacity_change > b->pickup_node->capacity_change;
}

void GreedyInsertion::Metrics::draw()
{
#ifndef SCFPDP_HEADLESS
	SolutionMetrics::draw();
	ImGui::Text("Served Requests: %zu", served_requests);
#endif // SCFPDP_HEADLESS
}

void GreedyInsertion::Metrics::print_head(std::ostream& os) const
{
	SolutionMetrics::print_head(os);
	os << ",served_requests";
}

void GreedyInsertion::Metrics::print(std::ostream& os) const
{
	SolutionMetrics::print(os);
	os << "," << served_requests;
}

GreedyInsertion::GreedyInsertion(Instance* instance)
	: instance(instance), initial_solution(instance, true), state(initial_solution)
{
	reset();
}

GreedyInsertion::GreedyInsertion(Instance* instance, const Solution& initial_solution)
	: instance(instance), initial_solution(initial_solution), state(this->initial_solution)
{
	reset();
}

Solution GreedyInsertion::get_current_solution()
{
	Solution sol(instance, true);
	for (size_t i = 0; i < state.routes.size(); ++i)
	{
		Route& route = sol.routes.at(i);
		for (const auto& [node, load] : state.routes[i])
			route.nodes.push_back(node);
		route.total_distance = instance->calculate_route_time(route);
	}
	metrics.step = state.step;
	metrics.served_requests = state.served_requests;
	sol.calculate_metrics(metrics);
	// todo: delta seems broken
	//metrics.total_cost = state.delta.cost();
	//metrics.fairness = state.delta.fairness();
	return sol;
}

size_t GreedyInsertion::step()
{
	State& s = state;
	if (s.done)
		return s.step;

	// convenience
	auto unserved_empty = [&]() -> bool {
		return s.unserved_cursor >= s.unserved_sorted.size();
	};

	// If nothing is left to insert, we’re done.
	if (unserved_empty())
	{
		s.done = true;
		return s.step;
	}

	// Phase 1: seed empty routes with 1 request each
	// (keeps your original behaviour: initialise each empty route with one request).
	if (s.served_requests == 0)
	{
		for (size_t i = 0; i < s.routes.size() && !unserved_empty(); ++i)
		{
			auto& route = s.routes[i];
			if (!route.empty())
				continue;

			CustomerRequest* req = s.unserved_sorted[s.unserved_cursor++];
			Node* cr_pickup  = req->pickup_node;
			Node* cr_dropoff = req->dropoff_node;
			const int request_load = cr_pickup->capacity_change;

			route.emplace_back(cr_pickup, request_load);
			route.emplace_back(cr_dropoff, 0);

			// delta updates
			s.delta.add_stop(i, nullptr, cr_pickup);
			s.delta.add_stop(i, cr_pickup, cr_dropoff);

			++s.served_requests;
		}
		return ++s.step;
	}

	// Phase 2: insert next request (in NodeOrder) into the closest feasible position.
	CustomerRequest* req = s.unserved_sorted[s.unserved_cursor];
	Node* cr_pickup  = req->pickup_node;
	Node* cr_dropoff = req->dropoff_node;
	const int request_load = cr_pickup->capacity_change;

	// Find closest feasible pickup insertion point (scan all nodes of all routes).
	// We preserve the rule “insert pickup after the closest feasible node”.
	struct BestPickup
	{
		size_t route_idx = std::numeric_limits<size_t>::max();
		State::Route::iterator after_it; // insert AFTER this node
		dist_t dist = std::numeric_limits<dist_t>::max();
		bool found = false;
	};

	BestPickup best;
	for (size_t r = 0; r < s.routes.size(); ++r)
	{
		auto& route = s.routes[r];
		if (route.empty())
			continue; // routes are seeded already; keep this simple

		for (auto it = route.begin(); it != route.end(); ++it)
		{
			// immediate feasibility: pickup load right after insertion point
			const int p_new_load = it->second + request_load;
			if (p_new_load > instance->vehicle_capacity)
				continue;

			const dist_t d = instance->get_distance(cr_pickup, it->first);
			if (d < best.dist)
			{
				best.dist = d;
				best.route_idx = r;
				best.after_it = it;
				best.found = true;
			}
		}
	}

	if (!best.found)
	{
		// No feasible pickup insertion (e.g. request_load too large for any position).
		// Mark done to avoid an infinite loop.
		s.done = true;
		return s.step;
	}

	auto& route = s.routes[best.route_idx];
	const int p_new_load = best.after_it->second + request_load;

	// Insert pickup after best.after_it
	const auto p_inserted_it = route.insert(std::next(best.after_it), {cr_pickup, p_new_load});
	{
		const auto p_next_it = std::next(p_inserted_it);
		s.delta.add_stop(
			best.route_idx,
			best.after_it->first,
			cr_pickup,
			p_next_it != route.end() ? p_next_it->first : nullptr);
	}

	// Choose dropoff insertion point.
	// NOTE: Current implementation copies DeltaCalculator per candidate.
	// If this is a bottleneck, consider adding a "try_add_stop / undo" or
	// "eval_add_stop" API to DeltaCalculator to avoid repeated deep copies.
	float best_cost = std::numeric_limits<float>::max();
	auto best_dropoff_it = route.end();

	for (auto dropoff_it = std::next(p_inserted_it); dropoff_it != route.end(); ++dropoff_it)
	{
		// We can always drop off immediately after pickup.
		// Past a point where capacity would be violated, later positions will also violate.
		if (dropoff_it->second + request_load > instance->vehicle_capacity)
			break;

		DeltaCalculator temp_delta = s.delta;
		const auto next_it = std::next(dropoff_it);
		temp_delta.add_stop(
			best.route_idx,
			dropoff_it->first,
			cr_dropoff,
			next_it != route.end() ? next_it->first : nullptr);

		const float cost = temp_delta.cost();
		if (cost < best_cost)
		{
			best_cost = cost;
			best_dropoff_it = dropoff_it;
		}
	}

	if (best_dropoff_it == route.end())
	{
		// Only feasible: dropoff immediately after pickup
		const auto d_inserted_it = route.insert(std::next(p_inserted_it), {cr_dropoff, p_new_load - request_load});
		const auto d_next_it = std::next(d_inserted_it);
		s.delta.add_stop(
			best.route_idx,
			p_inserted_it->first,
			cr_dropoff,
			d_next_it != route.end() ? d_next_it->first : nullptr);
	}
	else
	{
		// Insert at best found position
		const auto d_inserted_it = route.insert(std::next(best_dropoff_it), {cr_dropoff, best_dropoff_it->second});
		const auto d_next_it = std::next(d_inserted_it);
		s.delta.add_stop(
			best.route_idx,
			best_dropoff_it->first,
			cr_dropoff,
			d_next_it != route.end() ? d_next_it->first : nullptr);

		// Update loads between pickup and dropoff
		for (auto it = std::next(p_inserted_it); it != d_inserted_it; ++it)
			it->second += request_load;
	}

	// Consume this request (O(1))
	++s.unserved_cursor;
	++s.served_requests;
	++s.step;

	if (s.served_requests >= instance->min_requests || unserved_empty())
		s.done = true;

	return s.step;
}

bool GreedyInsertion::done() { return state.done; }

void GreedyInsertion::reset()
{
	state.step = 0;
	state.done = false;
	state.served_requests = 0;
	state.routes.clear();

	state.unserved_sorted.clear();
	state.unserved_cursor = 0;

	state.delta = DeltaCalculator(initial_solution);

	assert(initial_solution.validate(true));

	// Rebuild routes from initial_solution and mark which requests are already served.
	const size_t n = instance->requests.size();
	static thread_local std::vector<unsigned char> served;
	served.resize(n);
	std::fill(served.begin(), served.end(), 0);

	state.routes.reserve(initial_solution.routes.size());
	for (const auto& r : initial_solution.routes)
	{
		State::Route new_route;
		int load = 0;
		for (Node* node : r.nodes)
		{
			load += node->capacity_change;
			state.delta.add_stop(state.routes.size(), new_route.empty() ? nullptr : new_route.back().first, node);
			new_route.emplace_back(node, load);

			if (node->is_pickup())
			{
				CustomerRequest& req = instance->get_request_from_node(node);
				const size_t idx = static_cast<size_t>(&req - &instance->requests[0]);
				if (idx < n) served[idx] = 1;
			}
			else
			{
				// counts requests already present in initial solution
				++state.served_requests;
			}
		}
		state.routes.push_back(std::move(new_route));
	}

	// Build sorted list of unserved requests (no map allocations).
	state.unserved_sorted.reserve(n);
	for (size_t i = 0; i < n; ++i)
	{
		if (!served[i])
			state.unserved_sorted.push_back(&instance->requests[i]);
	}

	// Keep the same ordering semantics as before.
	std::sort(state.unserved_sorted.begin(), state.unserved_sorted.end(), State::NodeOrder{});

	if (state.unserved_sorted.empty())
		state.done = true;
}

void GreedyInsertion::draw()
{
#ifndef SCFPDP_HEADLESS
	if (ImGui::Button("Load Solution from File"))
	{
		if (std::filesystem::exists("solutions/current_solution.sln"))
		{
			initial_solution = Solution::load_from_file("solutions/current_solution.sln", instance);
		}
	}
#endif // SCFPDP_HEADLESS
}
