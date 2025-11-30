#include "heuristics/neighborhood.h"
#include "iostream"
#include "neighborhood.h"


#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>

void apply_move_to_solution(Solution& sol, int from_driver_id, int to_driver_id,
	const Node* pickupNode, const Node* dropoffNode, Node* p_before, Node* p_after, Node* d_before,
	Node* d_after);

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

MoveRequestWithinRouteNeighborhood::NodeMover::NodeMover(const std::vector<Node*>& route,
	int route_id, size_t idx, DeltaCalculator delta, int load, int max_load, Node* terminator)
	: route(route)
	, route_id(route_id)
	, idx(idx)
	, delta(delta)
	, load(load)
	, max_load(max_load)
	, terminator(terminator)
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
	delta.swap_stops(route_id, i_a == 0 ? nullptr : route[i_left], route[i_a], route[i_b],
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
	delta.swap_stops(route_id, i_a == 0 ? nullptr : route[i_left], route[i_a], route[i_b],
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
	const auto pickup_it =
		std::find(dq.route_copy.begin(), dq.route_copy.end(), request->pickup_node);
	const auto dropoff_it =
		std::find(dq.route_copy.begin(), dq.route_copy.end(), request->dropoff_node);

	assert(pickup_it != dq.route_copy.end());
	assert(dropoff_it != dq.route_copy.end());

	const auto pickup_idx = std::distance(dq.route_copy.begin(), pickup_it);
	const auto dropoff_idx = std::distance(dq.route_copy.begin(), dropoff_it);

	assert(pickup_idx < dropoff_idx);

	// initialize left and right movers
	const int route_id = dq.driver_id;
	const int load_at_pickup = dq.load_at_position[pickup_idx];
	const int load_at_dropoff = dq.load_at_position[dropoff_idx];
	const int max_load = instance_->vehicle_capacity;
	// left mover moves dropoff to the left: terminator at pickup
	movers_[0] = std::make_unique<NodeMover>(dq.route_copy, route_id, dropoff_idx, initial_delta_,
		load_at_dropoff, max_load, *pickup_it);
	// right mover moves pickup to the right: terminator at dropoff
	movers_[1] = std::make_unique<NodeMover>(
		dq.route_copy, route_id, pickup_idx, initial_delta_, load_at_pickup, max_load, *dropoff_it);

	last_mover_ = 0;

	if (dq.queue.empty())
		request_queue_.pop();
}

MoveRequestWithinRouteNeighborhood::MoveRequestWithinRouteNeighborhood(
	const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution)
	, instance_(instance)
	, initial_solution_(initial_solution)
	, initial_delta_(instance, initial_solution)
{
	done_ = false;
	// initialize the request queue
	for (size_t driver_id = 0; driver_id < initial_solution_.routes.size(); ++driver_id)
	{
		DriverQueue dq;
		dq.driver_id = driver_id;
		dq.route_copy = std::vector<Node*>(initial_solution_.routes[driver_id].nodes.begin(),
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

	const bool success =
		this_mover == 0 ? movers_[this_mover]->swap_left() : movers_[this_mover]->swap_right();

	if (!success)
	{
		movers_[this_mover] = nullptr;
		return next();
	}

	last_mover_ = this_mover;

	const float cost_from_delta = movers_[this_mover]->delta.cost();
	// const float cost_from_soln = get().calculate_cost();
	// std::cout << "Current cost from delta: " << cost_from_delta
	// 	<< ", from solution: " << cost_from_soln << std::endl;
	return cost_from_delta;
}

Solution MoveRequestWithinRouteNeighborhood::get()
{
	if (movers_[last_mover_] == nullptr)
	{
		std::cout << "Warning: get() called but no move was made!" << std::endl;
		return initial_solution_; // should not happen
	}

	NodeMover& mover = *movers_[last_mover_];
	const auto route_id = mover.route_id;

	Solution sol = initial_solution_;
	sol.routes[route_id].nodes = std::list<Node*>(mover.route.begin(), mover.route.end());
	sol.routes[route_id].total_distance = sol.calculate_route_time(sol.routes[route_id]);
	return sol;
}

bool MoveRequestWithinRouteNeighborhood::has_next() { return !done_; }


//=============================================================


MoveRequestToAnotherRouteNeighborhood::MoveRequestToAnotherRouteNeighborhood(
	const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution)
	, current_solution_(initial_solution)
	, candidate_solution_(initial_solution)
	, delta_(instance, initial_solution)
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
			if (!node->is_pickup())
				continue;

			CustomerRequest& request = instance->get_request_from_node(node);
			request_queue_.emplace(&request, driver_id);
		}
	}
}
float MoveRequestToAnotherRouteNeighborhood::next()
{
	// try requests until we find one with a feasible move, or we run out.
	// each call to next() will advance the queue by processing (and popping) at least one request.
	while (!request_queue_.empty())
	{
		// peek and immediately pop the current request to advance the queue
		auto front_pair = request_queue_.front();
		request_queue_.pop();
		auto [request, from_driver_id] = front_pair;

		const Node* pickupNode = request->pickup_node;
		const Node* dropoffNode = request->dropoff_node;

		// try moving this request to every other driver
		for (size_t to_driver_id = 0; to_driver_id < current_solution_.routes.size();
			++to_driver_id)
		{
			if (to_driver_id == static_cast<size_t>(from_driver_id))
				continue;

			const auto& to_route = current_solution_.routes[to_driver_id];
			const auto& from_route = current_solution_.routes[from_driver_id];

			// copy node lists into vectors for indexing
			std::vector<Node*> to_nodes(to_route.nodes.begin(), to_route.nodes.end());
			std::vector<Node*> from_nodes(from_route.nodes.begin(), from_route.nodes.end());

			// load computation on target route
			std::vector<int> load;
			load.reserve(to_nodes.size() + 1);
			load.push_back(0);
			int cur = 0;
			for (auto n : to_nodes)
			{
				cur += n->capacity_change;
				load.push_back(cur);
			}

			// all pickup insertion points
			for (size_t p_pos = 0; p_pos <= to_nodes.size(); ++p_pos)
			{
				for (size_t d_pos = p_pos + 1; d_pos <= to_nodes.size(); ++d_pos)
				{
					// capacity check between pickup and dropoff
					int pickup_load = pickupNode->capacity_change;
					bool cap_ok = true;
					for (size_t t = 0; t <= to_nodes.size(); ++t)
					{
						int l = load[t];
						if (t >= p_pos)
							l += pickup_load;
						if (t >= d_pos)
							l -= pickup_load;

						if (l < 0 || l > instance_->vehicle_capacity)
						{
							cap_ok = false;
							break;
						}
					}
					if (!cap_ok)
						continue;

					if (std::find(to_nodes.begin(), to_nodes.end(), const_cast<Node*>(pickupNode))
						!= to_nodes.end())
						continue;
					if (std::find(to_nodes.begin(), to_nodes.end(), const_cast<Node*>(dropoffNode))
						!= to_nodes.end())
						continue;

					DeltaCalculator delta(instance_, current_solution_);

					// insertion of pickup
					Node* p_before = (p_pos == 0 ? nullptr : to_nodes[p_pos - 1]);
					Node* p_after = (p_pos == to_nodes.size() ? nullptr : to_nodes[p_pos]);

					delta.add_stop(to_driver_id, p_before, const_cast<Node*>(pickupNode), p_after);

					// safe insertion of dropoff
					Node* d_before = (d_pos == 0 ? nullptr : to_nodes[d_pos - 1]);
					Node* d_after = (d_pos == to_nodes.size() ? nullptr : to_nodes[d_pos]);

					delta.add_stop(to_driver_id, d_before, const_cast<Node*>(dropoffNode), d_after);

					// remove from source route
					auto it_p = std::find(
						from_nodes.begin(), from_nodes.end(), const_cast<Node*>(pickupNode));
					auto it_d = std::find(
						from_nodes.begin(), from_nodes.end(), const_cast<Node*>(dropoffNode));

					if (it_p == from_nodes.end() || it_d == from_nodes.end())
						continue;

					size_t ip = static_cast<size_t>(std::distance(from_nodes.begin(), it_p));
					size_t id = static_cast<size_t>(std::distance(from_nodes.begin(), it_d));

					Node* rp_before = (ip == 0 ? nullptr : from_nodes[ip - 1]);
					Node* rp_after = (ip + 1 >= from_nodes.size() ? nullptr : from_nodes[ip + 1]);

					Node* rd_before = (id == 0 ? nullptr : from_nodes[id - 1]);
					Node* rd_after = (id + 1 >= from_nodes.size() ? nullptr : from_nodes[id + 1]);

					if (id > ip)
					{
						delta.remove_stop(
							from_driver_id, rd_before, const_cast<Node*>(dropoffNode), rd_after);
						delta.remove_stop(
							from_driver_id, rp_before, const_cast<Node*>(pickupNode), rp_after);
					}
					else
					{
						delta.remove_stop(
							from_driver_id, rp_before, const_cast<Node*>(pickupNode), rp_after);
						delta.remove_stop(
							from_driver_id, rd_before, const_cast<Node*>(dropoffNode), rd_after);
					}

					// log last move
					last_move_ = {from_driver_id, static_cast<int>(to_driver_id), pickupNode,
						dropoffNode, p_before, p_after, d_before, d_after, true};

					// found a feasible move—return its cost immediately
					return delta.cost();
				}
			}
		}

		// no feasible move for this request
	}

	// no more neighbours
	done_ = true;
	last_move_.valid = false;
	return std::numeric_limits<float>::infinity();
}


Solution MoveRequestToAnotherRouteNeighborhood::get()
{
	if (!last_move_.valid)
	{
		return current_solution_;
	}

	Solution sol = current_solution_;
	apply_move_to_solution(sol, last_move_.from_driver_id, last_move_.to_driver_id,
		last_move_.pickupNode, last_move_.dropoffNode, last_move_.p_before, last_move_.p_after,
		last_move_.d_before, last_move_.d_after);


	return sol;
}

bool MoveRequestToAnotherRouteNeighborhood::has_next() { return !done_; }

void apply_move_to_solution(Solution& sol, int from_driver_id, int to_driver_id,
	const Node* pickupNode, const Node* dropoffNode, Node* p_before, Node* p_after, Node* d_before,
	Node* d_after)
{
	// --- REMOVE from old route ---
	auto& from_list = sol.routes[from_driver_id].nodes;
	from_list.remove(const_cast<Node*>(pickupNode));
	from_list.remove(const_cast<Node*>(dropoffNode));

	// --- INSERT pickup ---
	auto& to_list = sol.routes[to_driver_id].nodes;

	auto insert_after = [&](Node* before, Node* node)
	{
		if (before == nullptr)
		{
			to_list.push_front(node);
			return;
		}
		auto it = std::find(to_list.begin(), to_list.end(), before);
		++it;
		to_list.insert(it, node);
	};

	insert_after(p_before, const_cast<Node*>(pickupNode));
	insert_after(d_before, const_cast<Node*>(dropoffNode));

	// --- Recompute route distances (you don’t care about efficiency) ---
	sol.routes[from_driver_id].total_distance =
		sol.calculate_route_time(sol.routes[from_driver_id]);

	sol.routes[to_driver_id].total_distance = sol.calculate_route_time(sol.routes[to_driver_id]);
}