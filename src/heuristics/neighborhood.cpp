#include "heuristics/neighborhood.h"
#include "iostream"
#include "neighborhood.h"


#include <algorithm>
#include <cassert>
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

MoveRequestWithinRouteNeighborhood::NodeMover::NodeMover(const std::vector<Node*>& route,
	int route_id, size_t idx, DeltaCalculator delta, int load, int max_load, Node* terminator)
	: route(route)
	, route_id(route_id)
	, idx(idx)
	, terminator(terminator)
	, delta(delta)
	, load(load)
	, max_load(max_load)
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
	sol.routes[route_id].total_distance = instance_->calculate_route_time(sol.routes[route_id]);
	return sol;
}

bool MoveRequestWithinRouteNeighborhood::has_next() { return !done_; }


//=============================================================


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
		sol.instance->calculate_route_time(sol.routes[from_driver_id]);

	sol.routes[to_driver_id].total_distance = sol.instance->calculate_route_time(sol.routes[to_driver_id]);
}


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
	Solution sol = current_solution_;
	if (!last_move_.valid)
	{
		return sol;
	}

	apply_move_to_solution(sol, last_move_.from_driver_id, last_move_.to_driver_id,
		last_move_.pickupNode, last_move_.dropoffNode, last_move_.p_before, last_move_.p_after,
		last_move_.d_before, last_move_.d_after);

	return sol;
}

bool MoveRequestToAnotherRouteNeighborhood::has_next() { return !done_; }


//=============================================================


SwapRequestsBetweenRoutesNeighborhood::SwapRequestsBetweenRoutesNeighborhood(
	const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution)
	, instance_(instance)
	, current_solution_(initial_solution)
	, candidate_solution_(initial_solution)
	, delta_(instance, initial_solution)
{
	node_count_ = instance->node_count;
	done_ = false;

	// initialize driver pair queue (only i < j to avoid duplicates)
	for (size_t i = 0; i < initial_solution.routes.size(); ++i)
	{
		for (size_t j = i + 1; j < initial_solution.routes.size(); ++j)
		{
			driver_pair_queue_.emplace(static_cast<int>(i), static_cast<int>(j));
		}
	}
	prepare_request_queues();
}

void SwapRequestsBetweenRoutesNeighborhood::prepare_request_queues()
{
	// Clear queues
	while (!from_driver_request_queue_.empty())
	{
		from_driver_request_queue_.pop();
	}
	while (!to_driver_request_queue_.empty())
	{
		to_driver_request_queue_.pop();
	}

	if (driver_pair_queue_.empty())
	{
		done_ = true;
		return;
	}

	auto driver_pair = driver_pair_queue_.front();
	driver_pair_queue_.pop();
	int from_id = driver_pair.first;
	int to_id = driver_pair.second;

	// Fill from_driver_request_queue_ with requests from from_id
	const Route& from_route = current_solution_.routes[from_id];
	for (const Node* node : from_route.nodes)
	{
		if (node->is_pickup())
		{
			CustomerRequest& request = instance_->get_request_from_node(node);
			from_driver_request_queue_.emplace(&request, from_id);
		}
	}

	// Fill to_driver_request_queue_ with requests from to_id
	const Route& to_route = current_solution_.routes[to_id];
	for (const Node* node : to_route.nodes)
	{
		if (node->is_pickup())
		{
			CustomerRequest& request = instance_->get_request_from_node(node);
			to_driver_request_queue_.emplace(&request, to_id);
		}
	}
}

float SwapRequestsBetweenRoutesNeighborhood::next()
{
	if (from_driver_request_queue_.empty() || to_driver_request_queue_.empty())
	{
		prepare_request_queues();
		if (done_)
		{
			last_move_A.valid = false;
			last_move_B.valid = false;
			return std::numeric_limits<float>::infinity();
		}
	}

	// Pop one request from each queue
	auto reqA_pair = from_driver_request_queue_.front();
	from_driver_request_queue_.pop();
	auto [reqA, from_id] = reqA_pair;

	auto reqB_pair = to_driver_request_queue_.front();
	to_driver_request_queue_.pop();
	auto [reqB, to_id] = reqB_pair;

	const Node* pickupNodeA = reqA->pickup_node;
	const Node* dropoffNodeA = reqA->dropoff_node;
	const Node* pickupNodeB = reqB->pickup_node;
	const Node* dropoffNodeB = reqB->dropoff_node;

	// Convert routes to vectors
	std::vector<Node*> from_nodes(current_solution_.routes[from_id].nodes.begin(),
		current_solution_.routes[from_id].nodes.end());
	std::vector<Node*> to_nodes(
		current_solution_.routes[to_id].nodes.begin(), current_solution_.routes[to_id].nodes.end());

	// Find removal positions for A in from_nodes
	auto it_pa = std::find(from_nodes.begin(), from_nodes.end(), const_cast<Node*>(pickupNodeA));
	auto it_da = std::find(from_nodes.begin(), from_nodes.end(), const_cast<Node*>(dropoffNodeA));
	size_t ipa = std::distance(from_nodes.begin(), it_pa);
	size_t ida = std::distance(from_nodes.begin(), it_da);

	// Find removal positions for B in to_nodes
	auto it_pb = std::find(to_nodes.begin(), to_nodes.end(), const_cast<Node*>(pickupNodeB));
	auto it_db = std::find(to_nodes.begin(), to_nodes.end(), const_cast<Node*>(dropoffNodeB));
	size_t ipb = std::distance(to_nodes.begin(), it_pb);
	size_t idb = std::distance(to_nodes.begin(), it_db);

	// Temporary routes after removal
	std::vector<Node*> temp_from;
	temp_from.reserve(from_nodes.size() - 2);
	for (auto n : from_nodes)
	{
		if (n != const_cast<Node*>(pickupNodeA) && n != const_cast<Node*>(dropoffNodeA))
		{
			temp_from.push_back(n);
		}
	}

	std::vector<Node*> temp_to;
	temp_to.reserve(to_nodes.size() - 2);
	for (auto n : to_nodes)
	{
		if (n != const_cast<Node*>(pickupNodeB) && n != const_cast<Node*>(dropoffNodeB))
		{
			temp_to.push_back(n);
		}
	}

	// Compute load profiles after removal
	std::vector<int> load_temp_from(temp_from.size() + 1);
	load_temp_from[0] = 0;
	int cur_load = 0;
	for (size_t k = 0; k < temp_from.size(); ++k)
	{
		cur_load += temp_from[k]->capacity_change;
		load_temp_from[k + 1] = cur_load;
	}

	std::vector<int> load_temp_to(temp_to.size() + 1);
	load_temp_to[0] = 0;
	cur_load = 0;
	for (size_t k = 0; k < temp_to.size(); ++k)
	{
		cur_load += temp_to[k]->capacity_change;
		load_temp_to[k + 1] = cur_load;
	}

	// Nested loops over insertion positions
	for (size_t p_pos_A = 0; p_pos_A <= temp_to.size(); ++p_pos_A)
	{
		for (size_t d_pos_A = p_pos_A + 1; d_pos_A <= temp_to.size(); ++d_pos_A)
		{
			// Capacity check for inserting A into temp_to
			int demand_A = pickupNodeA->capacity_change;
			bool cap_ok_A = true;
			for (size_t t = 0; t <= temp_to.size(); ++t)
			{
				int l = load_temp_to[t];
				if (t >= p_pos_A)
					l += demand_A;
				if (t >= d_pos_A)
					l -= demand_A;
				if (l < 0 || l > instance_->vehicle_capacity)
				{
					cap_ok_A = false;
					break;
				}
			}
			if (!cap_ok_A)
				continue;

			for (size_t p_pos_B = 0; p_pos_B <= temp_from.size(); ++p_pos_B)
			{
				for (size_t d_pos_B = p_pos_B + 1; d_pos_B <= temp_from.size(); ++d_pos_B)
				{
					// Capacity check for inserting B into temp_from
					int demand_B = pickupNodeB->capacity_change;
					bool cap_ok_B = true;
					for (size_t t = 0; t <= temp_from.size(); ++t)
					{
						int l = load_temp_from[t];
						if (t >= p_pos_B)
							l += demand_B;
						if (t >= d_pos_B)
							l -= demand_B;
						if (l < 0 || l > instance_->vehicle_capacity)
						{
							cap_ok_B = false;
							break;
						}
					}
					if (!cap_ok_B)
						continue;

					// Both insertions feasible; compute delta
					DeltaCalculator delta(instance_, current_solution_);

					// Remove A from from_route
					Node* pA_before_remove = (ipa == 0 ? nullptr : from_nodes[ipa - 1]);
					Node* pA_after_remove =
						(ipa + 1 == from_nodes.size() ? nullptr : from_nodes[ipa + 1]);
					delta.remove_stop(
						from_id, pA_before_remove, const_cast<Node*>(pickupNodeA), pA_after_remove);

					Node* dA_before_remove = (ida == 0 ? nullptr : from_nodes[ida - 1]);
					Node* dA_after_remove =
						(ida + 1 == from_nodes.size() ? nullptr : from_nodes[ida + 1]);
					delta.remove_stop(from_id, dA_before_remove, const_cast<Node*>(dropoffNodeA),
						dA_after_remove);

					// Remove B from to_route
					Node* pB_before_remove = (ipb == 0 ? nullptr : to_nodes[ipb - 1]);
					Node* pB_after_remove =
						(ipb + 1 == to_nodes.size() ? nullptr : to_nodes[ipb + 1]);
					delta.remove_stop(
						to_id, pB_before_remove, const_cast<Node*>(pickupNodeB), pB_after_remove);

					Node* dB_before_remove = (idb == 0 ? nullptr : to_nodes[idb - 1]);
					Node* dB_after_remove =
						(idb + 1 == to_nodes.size() ? nullptr : to_nodes[idb + 1]);
					delta.remove_stop(
						to_id, dB_before_remove, const_cast<Node*>(dropoffNodeB), dB_after_remove);

					// Insert A into to_route (using temp_to neighbors)
					Node* p_before_A = (p_pos_A == 0 ? nullptr : temp_to[p_pos_A - 1]);
					Node* p_after_A = (p_pos_A == temp_to.size() ? nullptr : temp_to[p_pos_A]);
					delta.add_stop(to_id, p_before_A, const_cast<Node*>(pickupNodeA), p_after_A);

					Node* d_before_A = (d_pos_A == 0 ? nullptr : temp_to[d_pos_A - 1]);
					Node* d_after_A = (d_pos_A == temp_to.size() ? nullptr : temp_to[d_pos_A]);
					delta.add_stop(to_id, d_before_A, const_cast<Node*>(dropoffNodeA), d_after_A);

					// Insert B into from_route (using temp_from neighbors)
					Node* p_before_B = (p_pos_B == 0 ? nullptr : temp_from[p_pos_B - 1]);
					Node* p_after_B = (p_pos_B == temp_from.size() ? nullptr : temp_from[p_pos_B]);
					delta.add_stop(from_id, p_before_B, const_cast<Node*>(pickupNodeB), p_after_B);

					Node* d_before_B = (d_pos_B == 0 ? nullptr : temp_from[d_pos_B - 1]);
					Node* d_after_B = (d_pos_B == temp_from.size() ? nullptr : temp_from[d_pos_B]);
					delta.add_stop(from_id, d_before_B, const_cast<Node*>(dropoffNodeB), d_after_B);

					// Record moves
					last_move_A = {from_id, to_id, pickupNodeA, dropoffNodeA, p_before_A, p_after_A,
						d_before_A, d_after_A, true};
					last_move_B = {to_id, from_id, pickupNodeB, dropoffNodeB, p_before_B, p_after_B,
						d_before_B, d_after_B, true};

					return delta.cost();
				}
			}
		}
	}

	// No feasible swap for this request pair; continue to next
	return next();
}

Solution SwapRequestsBetweenRoutesNeighborhood::get()
{
	Solution sol = current_solution_;
	if (!last_move_A.valid || !last_move_B.valid)
	{
		return sol;
	}

	apply_move_to_solution(sol, last_move_A.from_driver_id, last_move_A.to_driver_id,
		last_move_A.pickupNode, last_move_A.dropoffNode, last_move_A.p_before, last_move_A.p_after,
		last_move_A.d_before, last_move_A.d_after);
	apply_move_to_solution(sol, last_move_B.from_driver_id, last_move_B.to_driver_id,
		last_move_B.pickupNode, last_move_B.dropoffNode, last_move_B.p_before, last_move_B.p_after,
		last_move_B.d_before, last_move_B.d_after);

	return sol;
}

bool SwapRequestsBetweenRoutesNeighborhood::has_next() { return !done_; }