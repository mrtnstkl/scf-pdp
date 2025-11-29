#include <iostream>
#include <vector>
#include <string>
#include <array>

#include "core/instance.h"
#include "core/solution.h"
#include "heuristics/construction.h"
#include "heuristics/beamsearch.h"
#include "heuristics/localsearch.h"
#include "util/canvas.h"
#include "util/visualization.h"

#include "imgui.h"

int main(int argc, char* argv[])
{
	std::vector<std::string> args(argv, argv + argc);

	if (args.size() != 2)
	{
		std::cerr << "Usage: " << args[0] << " <instance_file>" << std::endl;
		return 1;
	}

	Instance instance = Instance::load_from_file(args[1].c_str());
	//instance.print();

	auto dcon = GreedyNearestAny(&instance);
	auto gcon = GreedyRandomizedClosest(&instance);
	auto wcon = GreedyNearestWeighted(&instance);
	auto rcon = RandomConstruction(&instance);
	auto bsearch = BeamSearch(&instance);
	auto search = LocalSearch(&instance);

	auto algorithms = std::array{
		AlgorithmGui("Greedy Nearest Any", &dcon),
		AlgorithmGui("Greedy Nearest Radnomized", &gcon),
		AlgorithmGui("Greedy Nearest Weighted", &wcon),
		AlgorithmGui("Randomized", &rcon),
		AlgorithmGui("Beam Search", &bsearch),
		AlgorithmGui("Local Search", &search)
	};

	Gui gui;
	if (!gui.init("SCF-PDP Solver", 1200, 900))
	{
		std::cerr << "Failed to initialize GUI." << std::endl;
		return 1;
	}

	AlgorithmGui* last_open = nullptr;
	AlgorithmGui* pending_open = &algorithms[0];
	while (!gui.should_close())
	{
		gui.push_element([&]() {

			for (auto& algo_gui : algorithms)
			{
				if (pending_open != &algo_gui && pending_open != nullptr)
					ImGui::SetNextItemOpen(false);
				if (ImGui::CollapsingHeader(algo_gui.name()))
				{
					if (last_open != &algo_gui)
						pending_open = &algo_gui;
					algo_gui.draw();
					last_open = &algo_gui;
				}
			}
		});
		gui.push_window(last_open ? last_open->name() : "Visualization");
		if (last_open != nullptr)
			gui.push_element(last_open->get_visualization_gui());
		gui.update();
	}
	gui.close();

	return 0;
}
