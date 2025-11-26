#include "localsearch.h"
#include "core/solution.h"


Neighborhood::Neighborhood(const Solution& initial_solution)
	: initial_solution_(initial_solution)
{
}

// Neighborhood 1: Move Request Within Route

MoveRequestWithinRouteNeighborhood::MoveRequestWithinRouteNeighborhood(const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution), delta_(instance, initial_solution), current_solution_(initial_solution)
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
Solution MoveRequestWithinRouteNeighborhood::get()
{
	return current_solution_;
}
bool MoveRequestWithinRouteNeighborhood::has_next()
{
	return !done_;
}


// Neighborhood 2: Swap Requests Between Vehicles



// Neighborhood 3: Move Request Between Vehicles

MoveRequestBetweenVehiclesNeighborhood::MoveRequestBetweenVehiclesNeighborhood(const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution), delta_(instance, initial_solution), current_solution_(initial_solution)
{

	node_count_ = instance->node_count;
	done_ = false;
	vehicle_capacity_ = instance->vehicle_capacity;

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

float MoveRequestBetweenVehiclesNeighborhood::next()
{
	auto [req, from_driver] = request_queue_.front();
	request_queue_.pop();

	for (size_t to_driver = 0; to_driver < current_solution_.routes.size(); ++to_driver)
	{
		// Skip the original driver
		if ((int)to_driver == (int)from_driver)
			continue;

		Route& target_route = current_solution_.routes[to_driver];

		size_t route_length = target_route.nodes.size();

		// Iterate over all possible insertion positions
		for (size_t i = 0; i <= route_length; ++i) // pickup position
		{
			for (size_t j = i + 1; j <= route_length + 1; ++j) // dropoff position
			{

				// --- GET ITERATORS FOR POSITION i and j ---
				auto it_pick = target_route.nodes.begin();
				std::advance(it_pick, i);

				auto it_drop = target_route.nodes.begin();
				std::advance(it_drop, j);

				// Check load feasibility
				if (!is_feasible_insertion(target_route, req->pickup_node, req->dropoff_node, i, j, vehicle_capacity_))
					continue;

				// --- PERFORM THE MOVE ---
				// 1. Remove from original route
				Route& from_route = current_solution_.routes[from_driver];
				auto it_from_pick = std::find(from_route.nodes.begin(), from_route.nodes.end(), req->pickup_node);
				auto it_from_drop = std::find(from_route.nodes.begin(), from_route.nodes.end(), req->dropoff_node);
				delta_.remove_stop(from_driver,
					(it_from_pick == from_route.nodes.begin()) ? nullptr : *(std::prev(it_from_pick)),
					req->pickup_node,
					(it_from_pick == std::prev(from_route.nodes.end())) ? nullptr : *(std::next(it_from_pick))
				);
				delta_.remove_stop(from_driver,
					(it_from_drop == from_route.nodes.begin()) ? nullptr : *(std::prev(it_from_drop)),
					req->dropoff_node,
					(it_from_drop == std::prev(from_route.nodes.end())) ? nullptr : *(std::next(it_from_drop))
				);
				from_route.nodes.erase(it_from_drop);
				from_route.nodes.erase(it_from_pick);	

				// 2. Insert into target route
				delta_.add_stop(to_driver,
					(it_pick == target_route.nodes.begin()) ? nullptr : *(std::prev(it_pick)),
					req->pickup_node,
					(it_pick == target_route.nodes.end()) ? nullptr : *it_pick
				);
				target_route.nodes.insert(it_pick, req->pickup_node);
				delta_.add_stop(to_driver,
					(it_drop == target_route.nodes.begin()) ? nullptr : *(std::prev(it_drop)),
					req->dropoff_node,
					(it_drop == target_route.nodes.end()) ? nullptr : *it_drop
				);
				target_route.nodes.insert(it_drop, req->dropoff_node);
				return delta_.cost();

			}
		}
	}


	return delta_.cost();

}