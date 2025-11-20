#include <iostream>
#include <vector>
#include <string>

#include "core/instance.h"
#include "core/solution.h"
#include "heuristics/construction.h"

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

	return 0;
}
