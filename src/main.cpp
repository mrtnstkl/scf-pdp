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
	Solution solution = construction::construct_deterministic(instance);
	solution.print();
	solution.write_to_file("out.txt");

	Canvas canvas;
	canvas.scale = 8.f;
	canvas.width = 70 * canvas.scale;
	canvas.height = 70 * canvas.scale;
	canvas.bg_color = COLOR_BLACK;
	canvas.stroke_width = .2f;
	solution.draw(canvas);

	Gui gui;
	if (!gui.init("SCF-PDP Solver", 600, 800))
	{
		std::cerr << "Failed to initialize GUI." << std::endl;
		return 1;
	}
	while (!gui.should_close())
	{
		gui.push_element(&canvas);
		gui.update();
	}
	gui.close();

	return 0;
}
