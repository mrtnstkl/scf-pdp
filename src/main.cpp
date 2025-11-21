#include <iostream>
#include <vector>
#include <string>

#include "core/instance.h"
#include "core/solution.h"
#include "heuristics/construction.h"
#include "util/canvas.h"

int main(int argc, char* argv[])
{
	std::vector<std::string> args(argv, argv + argc);

	if (args.size() != 2)
	{
		std::cerr << "Usage: " << args[0] << " <instance_file>" << std::endl;
		return 1;
	}

	Instance instance = Instance::load_from_file(args[1].c_str());
	instance.print();
	
	DeterministicConstructionHeuristic dcon(&instance);
	RandomConstruction rcon(&instance);

	Gui gui;
	if (!gui.init("SCF-PDP Solver", 600, 800))
	{
		std::cerr << "Failed to initialize GUI." << std::endl;
		return 1;
	}
	while (!gui.should_close())
	{
		gui.push_element(&dcon);
		gui.push_element(&rcon);
		gui.update();
	}
	gui.close();

	return 0;
}
