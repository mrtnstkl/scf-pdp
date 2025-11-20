#pragma once

#include <vector>
#include <list>

#include "instance.h"
#include "util/canvas.h"

struct Route
{
	Location depot; // the depot, which is always the start and end point of the route
	std::list<Node*> nodes; // the nodes in the route, excluding the depot

	int total_distance;
};

struct Solution
{
	Instance* instance;
	std::vector<Route> routes;

	Solution(Instance* instance, bool init = false);

	/** 
	 * @brief Calculate the total time (distance) of a given route
	 * 
	 * Uses the instance's distance matrix.
	 * TODO: maybe move this to Instance?
	 * 
	 * @param route The route to calculate the time for
	 * @return int The total time (distance) of the route
	 */
	int calculate_route_time(const Route& route) const;

	float calculate_jain_fairness() const;

	float calculate_cost() const;

	/** 
	 * @brief Validate the solution against instance constraints
	 * 
	 * @return true if the solution is valid
	 * @return false if the solution is invalid
	 */
	bool validate() const;

	void print() const;

	void draw(Canvas& canvas) const;

	void write_to_file(const char* filename, const char* comment = "solution") const;
};