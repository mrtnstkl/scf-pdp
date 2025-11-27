#include "heuristics/localsearch.h"
#include <fstream>
#include <iostream>
#include <sstream>

LocalSearch::LocalSearch(Instance *instance)
    : instance(instance), current_solution_(instance, false), finished(false) {}

void LocalSearch::reset()
{
    current_solution_ = Solution(instance);
    current_solution_.calculate_metrics(metrics);

    // Start algorithm logic at 0, but keep external load counter
    metrics.step = external_step_counter_++;     
    finished = false;

    // rebuild neighbourhood normally
    switch (selected_neighborhood_) {
        case NeighborhoodType::MoveWithinRoute:
            neighborhood_ = std::make_unique<MoveRequestWithinRouteNeighborhood>(current_solution_, instance);
            break;
        //case NeighborhoodType::SwapBetweenVehicles:
        //    neighborhood_ = std::make_unique<SwapRequestsBetweenVehiclesNeighborhood>(current_solution_, instance);
        //    break;
        case NeighborhoodType::MoveBetweenVehicles:
            neighborhood_ = std::make_unique<MoveRequestBetweenVehiclesNeighborhood>(current_solution_, instance);
            break;
    }
}


size_t LocalSearch::step()
{
    if (finished)
        return metrics.step;

    if (!neighborhood_ || !neighborhood_->has_next())
    {
        finished = true;
        return metrics.step;
    }

    // Apply one move
    neighborhood_->next();
    current_solution_ = neighborhood_->get();

    // Update metrics
    current_solution_.calculate_metrics(metrics);

    // 🔥 Give every step a NEW step ID
    metrics.step = external_step_counter_++;

    return metrics.step;
}


bool LocalSearch::done() { return finished; }

Solution LocalSearch::get_current_solution() { return current_solution_; }

void LocalSearch::draw() {
    if (ImGui::Button("Load current solution")) {
        load_solution_from_file("solutions/current_solution.sln");

        metrics.step = external_step_counter_++;

        finished = false;

        // Rebuild neighbourhood using the loaded solution
        switch (selected_neighborhood_) {
            case NeighborhoodType::MoveWithinRoute:
                neighborhood_ = std::make_unique<MoveRequestWithinRouteNeighborhood>(
                    current_solution_, instance);
                break;

            //case NeighborhoodType::SwapBetweenVehicles:
            //    neighborhood_ = std::make_unique<SwapRequestsBetweenVehiclesNeighborhood>(
            //        current_solution_, instance);
            //    break;

            case NeighborhoodType::MoveBetweenVehicles:
                neighborhood_ = std::make_unique<MoveRequestBetweenVehiclesNeighborhood>(
                    current_solution_, instance);
                break;
        }
    }

        ImGui::Text("Select Neighborhood:");
    if (ImGui::RadioButton("Move Within Route",
        selected_neighborhood_ == NeighborhoodType::MoveWithinRoute))
    {
        selected_neighborhood_ = NeighborhoodType::MoveWithinRoute;
        reset();
    }
    if (ImGui::RadioButton("Swap Between Vehicles",
        selected_neighborhood_ == NeighborhoodType::SwapBetweenVehicles))
    {
        selected_neighborhood_ = NeighborhoodType::SwapBetweenVehicles;
        reset();
    }
    if (ImGui::RadioButton("Move Between Vehicles",
        selected_neighborhood_ == NeighborhoodType::MoveBetweenVehicles))
    {
        selected_neighborhood_ = NeighborhoodType::MoveBetweenVehicles;
        reset();
    }


}

void LocalSearch::load_solution_from_file(const std::string& path)
{
    std::ifstream in(path);
    if (!in)
    {
        std::cerr << "Failed to open solution file: " << path << std::endl;
        return;
    }

    std::string header;
    std::getline(in, header);

    if (header != "solution")
    {
        std::cerr << "Invalid solution header in " << path << std::endl;
        return;
    }

    // Build a fresh solution
    current_solution_ = Solution(instance);

    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty())
            continue;

        std::istringstream ss(line);

        Route route;
        route.depot = instance->depot;

        int idx1based;
        while (ss >> idx1based)
        {
            // Convert 1-based index to internal 0-based index
            int idx0based = idx1based - 1;

            // Validate
            if (idx0based < 0 || idx0based >= (int)instance->node_count)
            {
                std::cerr << "Invalid node index " 
                          << idx1based 
                          << " in solution file (valid: 1–"
                          << instance->node_count << ")."
                          << std::endl;
                continue;
            }

            // Retrieve node
            Node* node = &instance->nodes[idx0based];
            route.nodes.push_back(node);
        }

        // Compute route distance and store
        route.total_distance = current_solution_.calculate_route_time(route);
        current_solution_.routes.push_back(std::move(route));
    }

    current_solution_.calculate_metrics(metrics);
    metrics.step = external_step_counter_++;
}
