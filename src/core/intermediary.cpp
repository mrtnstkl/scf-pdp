#include "core/intermediary.h"

#include "core/instance.h"

void Driver::visit(Node* node, Instance& instance)
{
	load += node->capacity_change;
	if (path.empty())
		time = instance.get_distance(node);
	else
		time += instance.get_distance(path.back(), node);
	if (node->is_pickup())
		loaded_requests.insert(&instance.get_request_from_node(node));
	else
		loaded_requests.erase(&instance.get_request_from_node(node));
	path.push_back(node);
}

bool Driver::fits(Node* node, Instance& instance) const
{
	if (node->is_pickup())
		return load + node->capacity_change <= instance.vehicle_capacity;
	else
	{
		CustomerRequest& cr = instance.get_request_from_node(node);
		return loaded_requests.contains(&cr);
	}
}
