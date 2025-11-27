#include "heuristics/localsearch.h"
#include <fstream>
#include <sstream>
#include <iostream>

LocalSearch::LocalSearch(Instance* instance)
    : instance(instance),
      current_solution_(instance)
{
    reset();
}

void LocalSearch::reset()
{
    // Start from an empty solution (or however you want)
    current_solution_ = Solution(instance);
    current_solution_.calculate_metrics(metrics);
    metrics.step = 0;

    finished = false;

    // Build selected neighbourhood from this starting solution
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

size_t LocalSearch::step()
{
    if (finished)
        return metrics.step;

    if (!neighborhood_ || !neighborhood_->has_next()) {
        finished = true;
        return metrics.step;
    }

    // Apply one neighbourhood move
    neighborhood_->next();
    current_solution_ = neighborhood_->get();

    // Update metrics for visualiser
    current_solution_.calculate_metrics(metrics);
    ++metrics.step;

    return metrics.step;
}

bool LocalSearch::done()
{
    return finished;
}

Solution LocalSearch::get_current_solution()
{
    return current_solution_;
}

void LocalSearch::load_solution_from_file(const std::string& path)
{
    std::ifstream in(path);
    if (!in) {
        std::cerr << "Failed to open solution file: " << path << std::endl;
        return;
    }

    std::string header;
    std::getline(in, header);
    if (header != "solution") {
        std::cerr << "Invalid solution header in " << path << std::endl;
        return;
    }

    current_solution_ = Solution(instance);

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty())
            continue;

        std::istringstream ss(line);
        Route route;
        route.depot = instance->depot;

        int idx1based;
        while (ss >> idx1based) {
            int idx0 = idx1based - 1;
            if (idx0 < 0 || idx0 >= (int)instance->node_count) {
                std::cerr << "Invalid node index " << idx1based
                          << " in solution file." << std::endl;
                continue;
            }
            Node* node = &instance->nodes[idx0];
            route.nodes.push_back(node);
        }

        route.total_distance = current_solution_.calculate_route_time(route);
        current_solution_.routes.push_back(std::move(route));
    }

    current_solution_.calculate_metrics(metrics);
    metrics.step = 0;      // loaded state = step 0
    finished = false;

    // Rebuild neighbourhood from loaded solution
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

void LocalSearch::draw()
{
    // Neighbourhood selection UI
    const char* names[] = {
        "Move Within Route",
        //"Swap Between Vehicles",
        "Move Between Vehicles"
    };
    int current = static_cast<int>(selected_neighborhood_);
    if (ImGui::Combo("Neighborhood", &current, names, IM_ARRAYSIZE(names))) {
        selected_neighborhood_ = static_cast<NeighborhoodType>(current);
        reset();
    }

    if (ImGui::Button("Load current solution")) {
        load_solution_from_file("solutions/current_solution.sln");
    }

    ImGui::Text("Step: %zu", metrics.step);
    ImGui::Text("Total Cost: %.2f", metrics.total_cost);
    ImGui::Text("Fairness: %.4f", metrics.fairness);
}
