#include "neighborhoods.h"
#include "core/solution.h"


Neighborhood::Neighborhood(const Solution& initial_solution)
	: initial_solution_(initial_solution)
{
}

// Neighborhood 1: Move Request Within Route

MoveRequestWithinRouteNeighborhood::MoveRequestWithinRouteNeighborhood(
	const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution)
	, delta_(instance, initial_solution)
	, current_solution_(initial_solution)
{
	node_count_ = instance->node_count;
	done_ = false;
	current_move_strategy_ = NEW_REQUEST;
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

float MoveRequestWithinRouteNeighborhood::next()
{
	bool new_found = false;
	while (!request_queue_.empty() && !new_found)
	{
		if (current_move_strategy_ == NEW_REQUEST)
		{
			// get a new request from the queue
			// move it to the very start
		}
	}

	return delta_.cost();
}
Solution MoveRequestWithinRouteNeighborhood::get() { return current_solution_; }
bool MoveRequestWithinRouteNeighborhood::has_next() { return !done_; }


// Neighborhood 2: Swap Requests Between Vehicles


// Neighborhood 3: Move Request Between Vehicles

MoveRequestBetweenVehiclesNeighborhood::MoveRequestBetweenVehiclesNeighborhood(
	const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution)
	, base_solution_(initial_solution)
	, candidate_solution_(initial_solution)
	, instance_(instance)
{
	node_count_ = instance->node_count;
	done_ = false;
	vehicle_capacity_ = instance->vehicle_capacity;

	// Build request queue from the base solution
	for (size_t driver_id = 0; driver_id < base_solution_.routes.size(); ++driver_id)
	{
		const Route& route = base_solution_.routes[driver_id];

		for (const Node* node : route.nodes)
		{
			if (node->is_pickup())
				continue; // only process each request once (we use dropoff as representative)
			CustomerRequest& request = instance_->get_request_from_node(node);
			request_queue_.emplace(&request, static_cast<int>(driver_id));
		}
	}
}


float MoveRequestBetweenVehiclesNeighborhood::next()
{
	// Find the next feasible move; skip requests with no feasible insertion
	while (!request_queue_.empty())
	{
		auto [req, from_driver] = request_queue_.front();
		
		// Iterate over all possible target vehicles
		for (size_t to_driver = 0; to_driver < base_solution_.routes.size(); ++to_driver)
		{
			if (static_cast<int>(to_driver) == from_driver)
				continue; // skip same vehicle

			const Route& target_route_base = base_solution_.routes[to_driver];
			size_t route_length = target_route_base.nodes.size();

			// Iterate over all possible insertion positions (pickup before dropoff)
			for (size_t i = 0; i <= route_length; ++i) // pickup position
			{
				for (size_t j = i + 1; j <= route_length + 1; ++j) // dropoff position
				{
					// Capacity / feasibility check on the base route
					if (!is_feasible_insertion(target_route_base, req->pickup_node,
							req->dropoff_node, i, j, vehicle_capacity_))
					{
						continue;
					}

					// --- BUILD CANDIDATE SOLUTION FROM BASE ---
					candidate_solution_ = base_solution_;

					// 1. Remove request from its original route in candidate
					Route& cand_from_route = candidate_solution_.routes[from_driver];

					auto it_from_pick = std::find(cand_from_route.nodes.begin(),
						cand_from_route.nodes.end(), req->pickup_node);
						
					auto it_from_drop = std::find(cand_from_route.nodes.begin(),
						cand_from_route.nodes.end(), req->dropoff_node);

					if (it_from_pick == cand_from_route.nodes.end()
						|| it_from_drop == cand_from_route.nodes.end())
					{
						// Should not happen if base_solution_ is consistent, skip this move
						continue;
					}

					// Always erase dropoff first, then re-find pickup
					cand_from_route.nodes.erase(it_from_drop);

					it_from_pick = std::find(cand_from_route.nodes.begin(),
						cand_from_route.nodes.end(), req->pickup_node);

					if (it_from_pick != cand_from_route.nodes.end())
					{
						cand_from_route.nodes.erase(it_from_pick);
					}


					// 2. Insert into target route in candidate
					Route& cand_target_route = candidate_solution_.routes[to_driver];

					// Recompute iterators for candidate target route
					auto cand_it_pick = cand_target_route.nodes.begin();
					std::advance(cand_it_pick, std::min(i, cand_target_route.nodes.size()));

					auto cand_it_drop = cand_target_route.nodes.begin();
					std::advance(cand_it_drop, std::min(j - 1, cand_target_route.nodes.size()));

					// Insert pickup
					cand_it_pick = cand_target_route.nodes.insert(cand_it_pick, req->pickup_node);

					// After inserting pickup, dropoff position might shift by +1 if j > i
					if (j <= cand_target_route.nodes.size())
					{
						cand_it_drop = cand_target_route.nodes.begin();
						std::advance(cand_it_drop, std::min(j, cand_target_route.nodes.size()));
					}
					else
					{
						cand_it_drop = cand_target_route.nodes.end();
					}

					cand_target_route.nodes.insert(cand_it_drop, req->dropoff_node);

					// simple (no delta) TODO use delta later
					SolutionMetrics m;
					candidate_solution_.calculate_metrics(m);
					return m.total_cost;				}
			}
		}
	}

	// No more moves available; mark as done.
	done_ = true;
	// Return the cost of the base solution
	SolutionMetrics m;
	base_solution_.calculate_metrics(m);
	return m.total_cost;
}


Solution MoveRequestBetweenVehiclesNeighborhood::get() { return candidate_solution_; }

bool MoveRequestBetweenVehiclesNeighborhood::has_next() { return !done_; }