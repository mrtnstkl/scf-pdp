#include "localsearch.h"
#include "neighborhood.h"

Neighborhood::Neighborhood(const Solution& initial_solution)
	: initial_solution_(initial_solution)
{
}

MoveRequestWithinRouteNeighborhood::MoveRequestWithinRouteNeighborhood(const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution), current_solution_(initial_solution), delta_(instance, initial_solution)
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
