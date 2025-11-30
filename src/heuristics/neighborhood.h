#pragma once

#include <list>
#include <memory>
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

// only for testing purposes
// returns the initial solution once
class DummyNeighborhood : public Neighborhood
{
	bool done_;

public:
	DummyNeighborhood(const Solution& initial_solution);
	float next() override;
	Solution get() override;
	bool has_next() override;
};

class MoveRequestWithinRouteNeighborhood : public Neighborhood
{
	Instance* instance_;
	bool done_;
	Solution initial_solution_;
	DeltaCalculator initial_delta_;

	struct NodeMover
	{
		std::vector<Node*> route;
		int route_id;
		size_t idx;
		Node* terminator;
		DeltaCalculator delta;
		int load, max_load;
		/**
		 * @brief Construct a new Node Mover object
		 *
		 * @param route the route to operate on
		 * @param node_id the id of the route
		 * @param idx the index of the node to move
		 * @param delta the delta calculator, initialized to the current solution
		 * @param load the load of the driver at idx (including the node at idx)
		 * @param max_load the maximum allowed load of the driver
		 * @param terminator the node at which to stop
		 */
		NodeMover(const std::vector<Node*>& route, int node_id, size_t idx, DeltaCalculator delta,
			int load, int max_load, Node* terminator = nullptr);
		/**
		 * @brief Swap the node at idx with the node to the left. Updates idx, load, and delta.
		 *
		 * @return false if not possible, either because idx is at start, terminator is reached, or
		 * capacity is violated
		 */
		bool swap_left();
		/**
		 * @brief Swap the node at idx with the node to the right. Updates idx, load, and delta.
		 *
		 * @return false if not possible, either because idx is at end, terminator is reached, or
		 * capacity is violated
		 */
		bool swap_right();
	};

	size_t last_mover_;
	std::unique_ptr<NodeMover> movers_[2] = {nullptr, nullptr};

	struct DriverQueue
	{
		int driver_id;
		std::queue<CustomerRequest*> queue; // requests for this driver
		std::vector<Node*> route_copy; // to avoid repeated conversions from list to vector
		std::vector<int> load_at_position; // precomputed load at every position
	};

	std::queue<DriverQueue> request_queue_;

	void prepare_next_request();

public:
	MoveRequestWithinRouteNeighborhood(const Solution& initial_solution, Instance* instance);
	float next() override;
	Solution get() override;
	bool has_next() override;
};


//=============================================================

class MoveRequestToAnotherRouteNeighborhood : public Neighborhood
{
	Instance* instance_;
	size_t node_count_;
	bool done_;
	Solution current_solution_;
	Solution candidate_solution_;
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
	int target_driver_id_;

	// store for each driver the current load at any given point in time
	std::vector<std::list<int>> driver_loads_;


	struct LastMove
	{
		int from_driver_id;
		int to_driver_id;
		const Node* pickupNode;
		const Node* dropoffNode;
		Node* p_before;
		Node* p_after;
		Node* d_before;
		Node* d_after;
		bool valid = false;
	};

	LastMove last_move_;

public:
	MoveRequestToAnotherRouteNeighborhood(const Solution& initial_solution, Instance* instance);
	float next() override;
	Solution get() override;
	bool has_next() override;
};


class SwapRequestsBetweenRoutesNeighborhood : public Neighborhood
{
	Instance* instance_;
	size_t node_count_;
	bool done_;
	Solution current_solution_;
	Solution candidate_solution_;
	DeltaCalculator delta_;

	struct LastMove
	{
		int from_driver_id;
		int to_driver_id;
		const Node* pickupNode;
		const Node* dropoffNode;
		Node* p_before;
		Node* p_after;
		Node* d_before;
		Node* d_after;
		bool valid = false;
	};

	LastMove last_move_A;
	LastMove last_move_B;

public:
	SwapRequestsBetweenRoutesNeighborhood(const Solution& initial_solution, Instance* instance);
	float next() override;
	Solution get() override;
	bool has_next() override;

	std::queue<std::pair<CustomerRequest*, int>> from_driver_request_queue_;
	std::queue<std::pair<CustomerRequest*, int>> to_driver_request_queue_;

private:
	void prepare_request_queues();
	std::queue<std::pair<int, int>> driver_pair_queue_;
};
