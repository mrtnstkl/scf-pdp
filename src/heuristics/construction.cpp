#include "construction.h"

#include <iostream>
#include <map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <ranges>

#include "../util/timer.hpp"

Solution construction::construct_deterministic(Instance& instance)
{
	std::cout << "Constructing initial solution...\n";

	Timer timer;

	struct Driver
	{
		int id;
		dist_t time = 0;
		int load = 0;
		std::set<CustomerRequest*> loaded_requests;
		std::vector<Node*> path;
	};

	// drivers sorted by their current time
	// this allows efficient retrieval of the driver that is furthest behind
	std::multimap<dist_t, Driver> drivers;

	std::unordered_set<Node*> unvisited_pickups;
	for (Node& pickup : instance.pickups)
		unvisited_pickups.insert(&pickup);

	// find the closest pickup locations to the depot
	{
		std::vector<Node*> pickups_near_depot = {unvisited_pickups.begin(), unvisited_pickups.end()};
		std::ranges::stable_sort(pickups_near_depot, [&](Node* a, Node* b)
			{ return instance.get_distance(a) > instance.get_distance(b); }
		);
		for (size_t i = 0; i < std::min<size_t>(instance.vehicle_count, pickups_near_depot.size()); ++i)
		{
			Node* pickup = pickups_near_depot[i]; // next closest pickup
			unvisited_pickups.erase(pickup); // remove from unvisited nodes
			Driver driver;
			driver.id = (int)i;
			driver.path.push_back(pickup); // add pickup to driver path
			driver.loaded_requests.insert(&instance.get_request_from_node(pickup)); // add request to driver
			driver.time = instance.get_distance(pickup);
			driver.load = pickup->capacity_change; 
			drivers.insert({driver.time, driver});
			//std::cout << "Driver " << driver.id << " starting with pickup at ("
			//	<< pickup->location.x << ", " << pickup->location.y << ") with distance "
			//	<< driver.time << "\n";
		}
	}

	size_t accepted_requests = 0;
	size_t served_requests = 0;
	const size_t request_target = instance.min_requests;

	std::vector<Driver> finished_drivers;
	while (!drivers.empty())
	{
		// select the driver that is furthest behind
		Driver driver = std::move(drivers.begin()->second);
		drivers.erase(drivers.begin());

		// find the closest node that is either...
		Node* closest_node = nullptr;
		uint16_t closest_dist = UINT16_MAX;
		// ...a dropoff for a loaded request...
		for (CustomerRequest* request : driver.loaded_requests)
		{
			const auto dist = instance.get_distance(request->dropoff_node, driver.path.back());
			if (dist < closest_dist)
			{
				closest_dist = dist;
				closest_node = request->dropoff_node;
			}
		}
		// ...or a pickup of a new one that fits (only if we still need more requests)
		if (accepted_requests < request_target)
		{
			for (Node* pickup : unvisited_pickups)
			{
				if (driver.load + pickup->capacity_change > instance.vehicle_capacity)
					continue; // would exceed capacity
				const auto dist = instance.get_distance(pickup, driver.path.back());
				if (dist < closest_dist)
				{
					closest_dist = dist;
					closest_node = pickup;
				}
			}
		}

		// no more suitible nodes, driver is finished
		if (closest_node == nullptr)
		{
			//std::cout << "Driver " << driver.id << " finished with "
			//	<< driver.path.size() << " nodes and time " << driver.time << "\n";
			finished_drivers.emplace_back(std::move(driver));
			continue;
		}

		//std::cout << "Driver " << driver.id << " @" << driver.time << " visiting node at ("
		//	<< closest_node->location.x << ", " << closest_node->location.y << ") "
		//	<< (closest_node->is_pickup() ? "pickup" : "dropoff") << " with distance "
		//	<< closest_dist << "\n";

		// add the node to the driver's path
		driver.time += closest_dist;
		driver.load += closest_node->capacity_change;
		driver.path.push_back(closest_node);
		if (closest_node->is_pickup())
		{
			unvisited_pickups.erase(closest_node);
			driver.loaded_requests.insert(&instance.get_request_from_node(closest_node));
		}
		else
		{
			driver.loaded_requests.erase(&instance.get_request_from_node(closest_node));
			served_requests++;
		}
		drivers.insert({driver.time, std::move(driver)});
	}

	timer.stop();
	std::cout << "Initial solution constructed in " << timer.get<std::string>() << "\n";

	Solution sol(&instance, false);
	for (auto& driver : finished_drivers)
	{
		Route route;
		route.depot = instance.depot;
		route.nodes = {driver.path.begin(), driver.path.end()};
		route.total_distance = sol.calculate_route_time(route);
		sol.routes.emplace_back(route);
	}
	return sol;
}