#pragma once

#include <list>
#include <queue>
#include <vector>

#include "core/algorithm.h"
#include "core/instance.h"
#include "core/solution.h"


class Neighborhood
{
protected:
	Solution initial_solution_;

public:
	Neighborhood(const Solution& initial_solution);
	virtual float next() = 0;
	virtual Solution get() = 0;
	virtual bool has_next() = 0;
};

class MoveRequestWithinRouteNeighborhood : public Neighborhood
{
	size_t node_count_;
	bool done_;
	Solution current_solution_;
	DeltaCalculator delta_;

	enum Moving
	{
		NEW_REQUEST,
		PICKUP,
		DROPOFF
	} current_move_strategy_;

	// the queue contains requests that shall be processed in order
	// along with the request, the driver id that has the request loaded is stored
	std::queue<std::pair<CustomerRequest*, int>> request_queue_;

	size_t pickup_index_, dropoff_index_;

	// store for each driver the current load at any given point in time
	std::vector<std::list<int>> driver_loads_;


public:
	MoveRequestWithinRouteNeighborhood(const Solution& initial_solution, Instance* instance);
	float next() override;
	Solution get() override;
	bool has_next() override;
};
