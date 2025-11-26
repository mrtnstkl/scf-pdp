#pragma once

#include <vector>
#include <queue>
#include <list>

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


// Neighborhood 1: Move Request Within Route
class MoveRequestWithinRouteNeighborhood : public Neighborhood
{
	size_t node_count_;
	bool done_;
	Solution current_solution_;
	DeltaCalculator delta_;

	enum Moving { NEW_REQUEST, PICKUP, DROPOFF } current_move_strategy_;

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


struct LocalSearch : public Algorithm
{
	Instance* instance;


	size_t step() override;
	void reset() override;
	bool done() override;
	
	Solution get_current_solution() override;

	void draw() override;


};

// Neighborhood 2: Swap Requests Between Vehicles

class SwapRequestsBetweenVehiclesNeighborhood : public Neighborhood
{

	size_t node_count_;
	bool done_;
	Solution current_solution_;
	DeltaCalculator delta_;

	std::queue<std::pair<CustomerRequest*, int>> request_queue_;

	// store for each driver the current load at any given point in time
	std::vector<std::list<int>> driver_loads_;

public:
	SwapRequestsBetweenVehiclesNeighborhood(const Solution& initial_solution, Instance* instance); 
	float next() override;
	Solution get() override;
	bool has_next() override;
};

// Neighborhood 3: Move Request Between Vehicles

class MoveRequestBetweenVehiclesNeighborhood : public Neighborhood
{
	size_t node_count_;
	bool done_;
	Solution current_solution_;
	DeltaCalculator delta_;
	int vehicle_capacity_;
	
	std::queue<std::pair<CustomerRequest*, int>> request_queue_;

	// store for each driver the current load at any given point in time
	std::vector<std::list<int>> driver_loads_;

public:
	MoveRequestBetweenVehiclesNeighborhood(const Solution& initial_solution, Instance* instance); 
	float next() override;
	Solution get() override;
	bool has_next() override;


	// Helper function to check load feasibility of an insertion by
	// simulating the load along the route and checking capacity constraints
	bool is_feasible_insertion(
	    const Route& route,
	    Node* pickup,
	    Node* dropoff,
	    size_t pos_pick,
	    size_t pos_drop,
	    int vehicle_capacity);
};