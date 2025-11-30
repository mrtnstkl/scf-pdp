#include "heuristics/neighborhood.h"

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


MoveRequestToAnotherRouteNeighborhood::MoveRequestToAnotherRouteNeighborhood(
	const Solution& initial_solution, Instance* instance)
	: Neighborhood(initial_solution)
	, delta_(instance, initial_solution)
	, current_solution_(initial_solution)
{
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

				//skip same route
				if (to_driver_id == from_driver_id)
					continue;

				// precompute weight at every position in to_route
				for (size_t i = 0; i < current_solution_.routes[to_driver_id].nodes.size(); ++i)
				{
					// init array of weights at every position between nodes
					//  routes have nodes, nodes have capacity_change
					//  jsut sum up capacity changes to get weight at every position

					std::vector<float> weight_at_position = {0};
					int current_weight = 0;

					for (auto node : current_solution_.routes[to_driver_id].nodes)
					{
						current_weight += node->capacity_change;
						weight_at_position.push_back(current_weight);
					}
				}

				const Route& from_route = current_solution_.routes[from_driver_id];
				// Try all insertion points in to_route
				


				return delta_.cost();
			}
		}

		Solution MoveRequestToAnotherRouteNeighborhood::get() { return current_solution_; }
		bool MoveRequestToAnotherRouteNeighborhood::has_next() { return !done_; }