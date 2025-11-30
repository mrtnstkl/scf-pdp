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