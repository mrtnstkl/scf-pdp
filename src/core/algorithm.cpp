#include "algorithm.h"

#include <chrono>
#include <functional>
#include <thread>

#include "imgui.h"

Algorithm::~Algorithm() { worker.stop(); }

Solution Algorithm::run(Algorithm& algorithm)
{ 
	algorithm.reset();
	while (!algorithm.done())
		algorithm.step();
	return algorithm.get_current_solution();
}

void Algorithm::Worker::start(std::function<void(const std::atomic_bool&)> func)
{
	stop();

	running = true;
	stop_requested = false;
	thread = std::thread(
		[this, func]()
		{
			func(stop_requested);
			running = false;
		});
}

void Algorithm::Worker::stop()
{
	if (thread.joinable())
	{
		stop_requested = true;
		thread.join();
	}
	running = false;
}


StoppingCriterion::StoppingCriterion() { reset(); }

bool StoppingCriterion::should_stop(float solution_cost)
{
	switch (type)
	{
	case TIME:
		if (!time_running)
		{
			time_timer.reset();
			time_running = true;
		}
		return time_timer.get<std::chrono::milliseconds>() >= time_limit_ms;
	case COST:
		return solution_cost <= cost_target;
	case HITS:
		return ++hits_current >= hits_limit;
	case STAGNATION:
		if (solution_cost < stagnation_best)
		{
			// improvement found
			stagnation_best = solution_cost;
			stagnation_counter = 0;
			return false;
		}
		else
		{
			// no improvement
			return ++stagnation_counter >= stagnation_limit;
		}
	default:
		break;
	}
}

void StoppingCriterion::draw()
{
	if (ImGui::SmallButton("Reset"))
		reset();

	static const char* type_names[] = { "Time", "Cost", "Hits", "Stagnation" };
	ImGui::Combo("Type", reinterpret_cast<int*>(&type), type_names, 4);

	switch (type)
	{
	case TIME:
		ImGui::InputScalar("Time Limit (ms)", ImGuiDataType_U64, &time_limit_ms);
		if (time_running)
			ImGui::Text("Elapsed Time: %s", time_timer.get<std::string>().c_str());
		else
			ImGui::Text("Elapsed Time: stopped");
		break;
	case COST:
		ImGui::InputScalar("Cost Target", ImGuiDataType_Float, &cost_target);
		break;
	case HITS:
		ImGui::InputScalar("Hits Limit", ImGuiDataType_U64, &hits_limit);
		ImGui::Text("Current Hits: %zu", hits_current);
		break;
	case STAGNATION:
		ImGui::InputScalar("Stagnation Limit", ImGuiDataType_U64, &stagnation_limit);
		ImGui::Text("Stagnation Counter: %zu", stagnation_counter);
		break;
	default:
		break;
	}
}

void StoppingCriterion::reset()
{
	hits_current = 0;
	stagnation_counter = 0;
	stagnation_best = std::numeric_limits<float>::max();
	time_running = false;
}
