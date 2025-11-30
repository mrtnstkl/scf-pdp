#include "heuristics/neighborhood.h"
#include "neighborhood.h"

#include <cassert>
#include <algorithm>
#include <iostream>
#include <numeric>

Neighborhood::Neighborhood(const Solution& initial_solution)
	: initial_solution_(initial_solution)
{
}

DummyNeighborhood::DummyNeighborhood(const Solution& initial_solution)
	: Neighborhood(initial_solution), done_(false)
{
}

float DummyNeighborhood::next()
{
	done_ = true;
	return initial_solution_.calculate_cost();
}

Solution DummyNeighborhood::get() { return initial_solution_; }

bool DummyNeighborhood::has_next() { return done_; }

MoveRequestWithinRouteNeighborhood::NodeMover::NodeMover(
	const std::vector<Node*>& route, int route_id, size_t idx, DeltaCalculator delta, int load, int max_load, Node* terminator)
	: route(route), route_id(route_id), idx(idx), delta(delta), load(load), max_load(max_load), terminator(terminator)
{
}

bool MoveRequestWithinRouteNeighborhood::NodeMover::swap_left()
{
	const size_t i_left = idx - 2;
	const size_t i_a = idx - 1;
	const size_t i_b = idx;
	const size_t i_right = idx + 1;

	// 1. check if possible (not at start, not at terminator)
	if (i_b == 0)
		return false;
	if (route[i_a] == terminator)
		return false;

	// 2. calculate new loads
	// since load is inclusive, swapping left does not change load at b, only at a
	const int new_load_at_a = load - route[i_a]->capacity_change;
	// 2a. check capacity
	if (new_load_at_a < 0 || new_load_at_a > max_load)
		return false;

	// 3. perform swap in delta calculator
	delta.swap_stops(
		route_id,
		i_a == 0 ? nullptr : route[i_left],
		route[i_a],
		route[i_b],
		i_right >= route.size() ? nullptr : route[i_right]);

	// 4. perform swap in route
	std::swap(route[i_a], route[i_b]);

	// 5. update idx and load
	idx -= 1;
	load = new_load_at_a;
	return true;
}

bool MoveRequestWithinRouteNeighborhood::NodeMover::swap_right()
{
	const size_t i_left = idx - 1;
	const size_t i_a = idx;
	const size_t i_b = idx + 1;
	const size_t i_right = idx + 2;

	// 1. check if possible (not at end, not at terminator)
	if (i_b >= route.size())
		return false;
	if (route[i_b] == terminator)
		return false;

	// 2. calculate new loads
	const int new_load_at_a = load - route[i_a]->capacity_change + route[i_b]->capacity_change;
	// since load is inclusive, swapping right changes load at a, but not at b
	// 2a. check capacity
	if (new_load_at_a < 0 || new_load_at_a > max_load)
		return false;

	// 3. perform swap in delta calculator
	delta.swap_stops(
		route_id,
		i_a == 0 ? nullptr : route[i_left],
		route[i_a],
		route[i_b],
		i_right >= route.size() ? nullptr : route[i_right]);

	// 4. perform swap in route
	std::swap(route[i_a], route[i_b]);

	// 5. update idx and load
	idx += 1;
	// load stays the same
	return true;
}


void MoveRequestWithinRouteNeighborhood::prepare_next_request()
{
	// get next request from queue
	assert(!request_queue_.empty());
	auto& dq = request_queue_.front();
	assert(!dq.queue.empty());
	auto* request = dq.queue.front();
	dq.queue.pop();

	// find the pickup and dropoff nodes in the route
	const auto pickup_it = std::find(dq.route_copy.begin(), dq.route_copy.end(), request->pickup_node);
	const auto dropoff_it = std::find(dq.route_copy.begin(), dq.route_copy.end(), request->dropoff_node);

	assert(pickup_it != dq.route_copy.end());
	assert(dropoff_it != dq.route_copy.end());

	const auto pickup_idx = std::distance(dq.route_copy.begin(), pickup_it);
	const auto dropoff_idx = std::distance(dq.route_copy.begin(), dropoff_it);

	// initialize left and right movers
	const int route_id = dq.driver_id;
	const int load_at_pickup = dq.load_at_position[pickup_idx];
	const int max_load = instance_->vehicle_capacity;
	// left mover: no terminator
	movers_[0] = std::make_unique<NodeMover>(
		dq.route_copy, route_id, pickup_idx, initial_delta_, load_at_pickup, max_load, nullptr);
	// right mover: terminator at dropoff
	movers_[1] = std::make_unique<NodeMover>(
		dq.route_copy, route_id, pickup_idx, initial_delta_, load_at_pickup, max_load, *dropoff_it);

	/*
	// initialize left and right movers
	const int route_id = dq.driver_id;
	const int load_at_dropoff = dq.load_at_position[dropoff_idx];
	const int max_load = instance_->vehicle_capacity;
	// left mover: no terminator
	movers_[0] = std::make_unique<NodeMover>(
		dq.route_copy, route_id, pickup_idx, initial_delta_, load_at_dropoff, max_load, *pickup_it);
	// right mover: terminator at dropoff
	movers_[1] = std::make_unique<NodeMover>(
		dq.route_copy, route_id, pickup_idx, initial_delta_, load_at_dropoff, max_load, nullptr);
	*/

	last_mover_ = 0;

	if (dq.queue.empty())
		request_queue_.pop();
}

MoveRequestWithinRouteNeighborhood::MoveRequestWithinRouteNeighborhood(
	const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution)
	, instance_(instance)
	, initial_delta_(instance, initial_solution)
	, initial_solution_(initial_solution)
{
	done_ = false;
	// initialize the request queue
	for (size_t driver_id = 0; driver_id < initial_solution_.routes.size(); ++driver_id)
	{
		DriverQueue dq;
		dq.driver_id = driver_id;
		dq.route_copy = std::vector<Node*>(
			initial_solution_.routes[driver_id].nodes.begin(),
			initial_solution_.routes[driver_id].nodes.end());

		// precompute load at every position
		dq.load_at_position = std::vector<int>(dq.route_copy.size());
		std::inclusive_scan(
			dq.route_copy.begin(), dq.route_copy.end(), dq.load_at_position.begin(),
			[](int sum, Node* node) { return sum + node->capacity_change; }, 0);

		// fill request queue
		for (const Node* node : dq.route_copy)
		{
			if (node->is_pickup())
				continue; // only process each request once, so skip pickups
			CustomerRequest& request = instance->get_request_from_node(node);
			dq.queue.emplace(&request);
		}
		if (!dq.queue.empty())
			request_queue_.emplace(std::move(dq));
	}
}

float MoveRequestWithinRouteNeighborhood::next()
{
	if (movers_[0] == nullptr && movers_[1] == nullptr)
	{
		if (request_queue_.empty())
		{
			done_ = true;
			return std::numeric_limits<float>::infinity();
		}
		prepare_next_request();
	}

	size_t this_mover = last_mover_ ^ 1;
	if (movers_[this_mover] == nullptr)
		this_mover = last_mover_;
	
	const bool success = this_mover == 0
		? movers_[this_mover]->swap_left()
		: movers_[this_mover]->swap_right();

	if (!success)
	{
		movers_[this_mover] = nullptr;
		return next();
	}

	last_mover_ = this_mover;
	return movers_[this_mover]->delta.cost();
}

Solution MoveRequestWithinRouteNeighborhood::get()
{
	if (movers_[last_mover_] == nullptr)
	{
		std::cout << "Warning: get() called but no move was made!" << std::endl;
		return initial_solution_; // should not happen
	}

	Solution sol = initial_solution_;
	const auto route_id = movers_[last_mover_]->route_id;

	std::cout
		<< "\nApplied move in route " << route_id << ", new cost: "
		<< movers_[last_mover_]->delta.cost() << std::endl;
	int i = 0;
	for (auto n : movers_[last_mover_]->route)
	{
		std::cout << (n->is_pickup() ? "P" : "D") << instance_->get_index(n) << " ";
	}


	sol.routes[route_id].nodes =
		std::list<Node*>(movers_[last_mover_]->route.begin(), movers_[last_mover_]->route.end());
	std::cout << "\nreal cost: " << sol.calculate_cost() << std::endl;
	std::cout << "route distance: " << sol.routes[route_id].total_distance << " -> " << sol.calculate_route_time(sol.routes[route_id]) << std::endl;
	sol.routes[route_id].total_distance = sol.calculate_route_time(sol.routes[route_id]);
	return sol;
}

bool MoveRequestWithinRouteNeighborhood::has_next() { return !done_; }


MoveRequestToAnotherRouteNeighborhood::MoveRequestToAnotherRouteNeighborhood(
	const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution)
	, delta_(instance, initial_solution)
	, current_solution_(initial_solution)
{
	instance_ = instance;
	node_count_ = instance->node_count;
	done_ = false;

	// initialize the request queue
	for (size_t driver_id = 0; driver_id < initial_solution_.routes.size(); ++driver_id)
	{
		const Route& route = initial_solution_.routes[driver_id];
		for (const Node* node : route.nodes)
		{
			if (node->is_pickup())
				continue; // only process each request once
			CustomerRequest& request = instance->get_request_from_node(node);
			request_queue_.emplace(&request, driver_id);
		}
	}
}
float MoveRequestToAnotherRouteNeighborhood::next()
{
	bool new_found = false;
	while (!request_queue_.empty() && !new_found)
	{
		// get a new request from the queue
		// try to move it to another route
		// Define route A and B
		// for each pickup in A:
		// go through the route and for each possible insertion point in B:
		// try to insert the pickup there and calculate where to
		// put the dropoff -> iff it gets close along the route there, if not then at overcapacity
		// point if cost is better, then store the move

		for (size_t i = 0; i < request_queue_.size(); ++i)
		{

			auto [request, from_driver_id] = request_queue_.front();
			auto pickupNode = request->pickup_node;
			auto dropoffNode = request->dropoff_node;
			auto weight = request->amount;


			for (size_t to_driver_id = 0; to_driver_id < current_solution_.routes.size();
				++to_driver_id)
			{
				std::vector<float> weight_at_position = {0};
				int current_weight = 0;


				// skip same route
				if (to_driver_id == from_driver_id)
					continue;

				// precompute weight at every position in to_route
				for (size_t i = 0; i < current_solution_.routes[to_driver_id].nodes.size(); ++i)
				{
					// init array of weights at every position between nodes
					//  routes have nodes, nodes have capacity_change
					//  jsut sum up capacity changes to get weight at every position


					for (auto node : current_solution_.routes[to_driver_id].nodes)
					{
						current_weight += node->capacity_change;
						weight_at_position.push_back(current_weight);
					}
				}

				const Route& from_route = current_solution_.routes[from_driver_id];
				// Try all insertion points in to_route
				auto pickup_iterator = current_solution_.routes[to_driver_id].nodes.begin();
				auto dropoff_iterator = current_solution_.routes[to_driver_id].nodes.begin();
				for (size_t pickup_pos = 0;
					pickup_pos < current_solution_.routes[to_driver_id].nodes.size();
					++pickup_pos, pickup_iterator++)
				{
					if (weight_at_position[pickup_pos] + weight > instance_->vehicle_capacity)
						continue; // skip infeasible insertion

					// find dropoff position
					size_t dropoff_pos = pickup_pos + 1;
					dropoff_iterator = std::next(pickup_iterator);

					for (; dropoff_pos
						< current_solution_.routes[to_driver_id].nodes.size() - pickup_pos;
						++dropoff_pos, dropoff_iterator++)
					{
						if (weight_at_position[dropoff_pos] + weight > instance_->vehicle_capacity)
							break;
					}
					// now we have pickup_pos and dropoff_pos
					// void add_stop(int route_id, const Node* prev_node, const Node* new_node,


					Node* insertAfter = *pickup_iterator;
					Node* insertBefore = *(std::next(pickup_iterator));

					delta_.add_stop(to_driver_id, insertAfter, pickupNode, insertBefore);

					Node* insertAfterDropoff = *dropoff_iterator;
					Node* insertBeforeDropoff = *(std::next(dropoff_iterator));

					delta_.add_stop(
						to_driver_id, insertAfterDropoff, dropoffNode, insertBeforeDropoff);

					// remove from from_route
					auto from_pickup_iterator =
						std::find(current_solution_.routes[from_driver_id].nodes.begin(),
							current_solution_.routes[from_driver_id].nodes.end(), pickupNode);

					auto from_dropoff_iterator =
						std::find(current_solution_.routes[from_driver_id].nodes.begin(),
							current_solution_.routes[from_driver_id].nodes.end(), dropoffNode);


					delta_.remove_stop(from_driver_id, *std::prev(from_pickup_iterator), pickupNode,
						*std::next(from_pickup_iterator));
					delta_.remove_stop(from_driver_id, *std::prev(from_dropoff_iterator), dropoffNode,
						*std::next(from_dropoff_iterator));


					break;
				}


				return delta_.cost();
			}
		}
	}
}

Solution MoveRequestToAnotherRouteNeighborhood::get() { return current_solution_; }
bool MoveRequestToAnotherRouteNeighborhood::has_next() { return !done_; }
