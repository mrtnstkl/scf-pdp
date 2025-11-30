#include  "heuristics/annealing.h"
#include "imgui.h"


Annealing
::Annealing(Instance* instance)
    : instance(instance),
      current_solution(instance),
      cooling_rate(cooling_rate)
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
    current_solution = Solution(instance);
    current_cost = current_solution.calculate_cost();
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
};