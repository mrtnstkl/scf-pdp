#include "construction.h"

#include <algorithm>
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
	static thread_local std::mt19937 rng(std::random_device{}());

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
	size_t index = rng() % closest.size();
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

	if (s.served_requests == 0)
	{
		// initialize routes with a single request each
		for (size_t i = 0; i < s.routes.size(); ++i)
		{
			auto& route = s.routes[i];
			// insert pickup and dropoff
			auto request_to_insert = s.unserved_requests.begin();
			Node* cr_pickup = request_to_insert->first->pickup_node;
			Node* cr_dropoff = request_to_insert->first->dropoff_node;
			const int request_load = cr_pickup->capacity_change;
			route.emplace_back(std::pair<Node*, int>(cr_pickup, request_load));
			route.emplace_back(std::pair<Node*, int>(cr_dropoff, 0));
			s.unserved_requests.erase(request_to_insert);
			// update delta calculator
			s.delta.add_stop(i, nullptr, cr_pickup);
			s.delta.add_stop(i, cr_pickup, cr_dropoff);
			++s.served_requests;
			// update distance maps of remaining unserved requests
			for (auto& [unserved_request, distance_map] : s.unserved_requests)
			{
				// for each remaining unserved request, add distances to the newly inserted nodes
				const dist_t dist_pickup = instance->get_distance(unserved_request->pickup_node, cr_pickup);
				const dist_t dist_dropoff =	instance->get_distance(unserved_request->pickup_node, cr_dropoff);
				distance_map.insert({dist_pickup, {s.routes.begin() + i, route.begin()}});
				distance_map.insert({dist_dropoff, {s.routes.begin() + i, std::next(route.begin())}});
			}
		}
		return ++s.step;
	}

	// we want to insert the front request of the unserved requests map
	auto request_to_insert = s.unserved_requests.begin();
	Node* cr_pickup = request_to_insert->first->pickup_node;
	Node* cr_dropoff = request_to_insert->first->dropoff_node;
	const int request_load = cr_pickup->capacity_change;

	//assert(request_to_insert->second.size() == s.served_requests * 2);

	// we want to insert the pickup after the closest feasible node
	for (auto& [dist, insertion_point] : request_to_insert->second)
	{
		const auto route_idx = std::distance(s.routes.begin(), insertion_point.route_it);
		auto& route = *insertion_point.route_it;
		// check if insertion would violate capacity
		const auto p_new_load = insertion_point.node_it->second + request_load;
		if (p_new_load > instance->vehicle_capacity)
			continue;
		// insert the pickup
		const auto p_inserted_it = route.insert(std::next(insertion_point.node_it), {cr_pickup, p_new_load});
		// update delta calculator
		const auto p_next_it = std::next(p_inserted_it);
		s.delta.add_stop(route_idx,
			insertion_point.node_it->first,
			cr_pickup,
			p_next_it != route.end() ? p_next_it->first : nullptr);

		// find insertion point for dropoff
		// for this, we scan the route from the pickup insertion point onwards, stopping at the first infeasible position
		// we then choose the best recorded position for insertion
		float best_cost = std::numeric_limits<float>::max();
		auto best_dropoff_it = route.end();

		for (auto dropoff_it = std::next(p_inserted_it); dropoff_it != route.end(); ++dropoff_it)
		{
			// check if insertion would violate capacity
			if (dropoff_it->second + request_load > instance->vehicle_capacity)
				break; // at this node, capacity would already be violated, so stop searching
			// simulate insertion and calculate cost
			DeltaCalculator temp_delta = s.delta;
			const auto next_it = std::next(dropoff_it);
			temp_delta.add_stop(route_idx, dropoff_it->first, cr_dropoff,
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
			// only possible insertion is right after pickup
			const auto d_inserted_it = route.insert(std::next(p_inserted_it), {cr_dropoff, p_new_load - request_load});
			// update delta calculator
			const auto d_next_it = std::next(d_inserted_it);
			s.delta.add_stop(route_idx, p_inserted_it->first, cr_dropoff,
				d_next_it != route.end() ? d_next_it->first : nullptr);
		}
		else
		{
			// insert at best found position
			const auto d_inserted_it = route.insert(std::next(best_dropoff_it), {cr_dropoff, best_dropoff_it->second});
			// update delta calculator
			const auto d_next_it = std::next(d_inserted_it);
			s.delta.add_stop(route_idx, best_dropoff_it->first, cr_dropoff,
				d_next_it != route.end() ? d_next_it->first : nullptr);
			// update loads between pickup and dropoff
			for (auto inbetween_it = std::next(p_inserted_it); inbetween_it != d_inserted_it; ++inbetween_it)
				inbetween_it->second += request_load;
		}
		// remove request from unserved requests
		s.unserved_requests.erase(request_to_insert);
		// update distance maps of remaining unserved requests
		for (auto& [unserved_request, distance_map] : s.unserved_requests)
		{
			// for each remaining unserved request, add distances to the newly inserted nodes
			const dist_t dist_pickup = instance->get_distance(unserved_request->pickup_node, cr_pickup);
			const dist_t dist_dropoff =	instance->get_distance(unserved_request->pickup_node, cr_dropoff);
			distance_map.insert({dist_pickup, {s.routes.begin() + route_idx, p_inserted_it}});
			distance_map.insert({dist_dropoff, {s.routes.begin() + route_idx, best_dropoff_it != route.end() ? best_dropoff_it : p_inserted_it}});
		}
		++s.served_requests;
		++s.step;
		break;
	}
	if (s.served_requests >= instance->min_requests || s.unserved_requests.empty())
	{
		s.done = true;
	}
	return s.step;
}

bool GreedyInsertion::done() { return state.done; }

void GreedyInsertion::reset()
{
	state.step = 0;
	state.done = false;
	state.served_requests = 0;
	state.routes.clear();
	state.unserved_requests.clear();
	state.delta = DeltaCalculator(initial_solution);

	assert(initial_solution.validate(true));

	for (CustomerRequest& request : instance->requests)
		// initialize all requests as unserved
		// the map of closest nodes is empty for now
		state.unserved_requests.insert({&request, {}});

	// populate routes and erase served requests from unserved map
	for (const auto& route : initial_solution.routes)
	{
		State::Route new_route;
		int load = 0;
		for (Node* node : route.nodes)
		{
			load += node->capacity_change;
			state.delta.add_stop(state.routes.size(), new_route.empty() ? nullptr : new_route.back().first, node);
			new_route.emplace_back(node, load);
			if (node->is_pickup())
				state.unserved_requests.erase(&instance->get_request_from_node(node));
			else
				state.served_requests++;
		}
		state.routes.push_back(std::move(new_route));
	}

	// compute distances of the pickups of unserved requests to all visited nodes
	for (auto& [request, distance_map] : state.unserved_requests)
	{
		for (auto route_it = state.routes.begin(); route_it != state.routes.end(); ++route_it)
		{
			for (auto node_it = route_it->begin(); node_it != route_it->end(); ++node_it)
			{
				const dist_t dist = instance->get_distance(request->pickup_node, node_it->first);
				distance_map.insert({dist, State::NearbyNode{route_it, node_it}});
			}
		}
	}

	if (state.unserved_requests.empty())
		state.done = true;

	//std::cout << "GreedyInsertion initialized with "
	//	<< state.routes.size() << " routes, "
	//	<< state.served_requests << " served requests, "
	//	<< state.unserved_requests.size() << " unserved requests.\n";
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
