#include  "heuristics/annealing.h"
#include "imgui.h"


Annealing
::Annealing(Instance* instance)
    : instance(instance),
      current_solution(instance),
      best_solution(instance),
      cooling_rate(0.99f)
      {
        reset();
};

size_t Annealing::step()
{
    return 0;
};

void Annealing::reset()
{
    initial_temperature = 0;
    cooling_rate = 0.99f;
    current_solution = Solution(instance);
    current_cost = current_solution.calculate_cost();

    best_solution = current_solution;
    best_cost = current_cost;
};

bool Annealing::done()
{
    return false;
};

Solution Annealing::get_current_solution()
{
    return current_solution;
};

void Annealing::draw()
{
    ImGui::SliderFloat("Initial Temperature", &initial_temperature, 0, 20);
    ImGui::SliderFloat("Cooling Rate", &cooling_rate, 0.1f, 0.999f);
};