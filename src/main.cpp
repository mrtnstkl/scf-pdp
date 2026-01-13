#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "core/instance.h"
#include "core/solution.h"
#include "heuristics/beamsearch.h"
#include "heuristics/construction.h"
#include "heuristics/grasp.h"
#include "heuristics/lns.h"
#include "heuristics/localsearch.h"
#include "heuristics/vnd.h"
#include "util/canvas.h"
#include "util/visualization.h"

#ifndef SCFPDP_HEADLESS
#include "imgui.h"

int gui_application(std::vector<std::string> args)
{
	Instance instance = Instance::load_from_file(args[1].c_str());

	auto dcon = GreedyNearestAny(&instance);
	auto gcon = GreedyRandomizedClosest(&instance);
	auto wcon = GreedyNearestWeighted(&instance);
	auto gins = GreedyInsertion(&instance);
	auto rcon = RandomConstruction(&instance);
	auto bsearch = BeamSearch(&instance);
	auto lsearch = LocalSearch(&instance);
	auto vnd = VariableNeighborhoodDescent(&instance);
	auto grasp = Grasp(&instance, std::make_unique<GreedyRandomizedClosest>(&instance), std::make_unique<LocalSearch>(&instance));
	auto lns = LargeNeighborhoodSearch(&instance);

	auto algorithms = std::array{
		AlgorithmGui("Greedy Nearest Any", &dcon),
		AlgorithmGui("Greedy Nearest Radnomized", &gcon),
		AlgorithmGui("Greedy Nearest Weighted", &wcon),
		AlgorithmGui("Greedy Insertion", &gins),
		AlgorithmGui("Randomized", &rcon),
		AlgorithmGui("Beam Search", &bsearch),
		AlgorithmGui("Local Search", &lsearch),
		AlgorithmGui("Variable Neighborhood Descent", &vnd),
		AlgorithmGui("GRASP", &grasp),
		AlgorithmGui("Large Neighborhood Search", &lns),
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
#endif // SCFPDP_HEADLESS


int main(int argc, char* argv[])
{
	std::vector<std::string> args(argv, argv + argc);
	for (auto& arg : args)
		std::cout << arg << " ";
	std::cout << std::endl;

	auto usage_greedy = args[0] + " greedy <instance>";
	auto usage_greedy_rand = args[0] + " greedy_rand <instance> <candidates> <tightening>";
	auto usage_random = args[0] + " random <instance>";
	auto usage_beam = args[0] + " beam <instance> <beam_width>";
	auto usage_localsearch = args[0] + " localsearch <instance> <initial_solution> <neighborhood_type> <strategy (\"first\"|\"best\")>";
	auto usage_vnd = args[0] + " vnd <instance> <initial_solution>";
	auto usage_grasp = args[0] + " grasp <instance> <candidates> <tightening> <neighborhood_type> <strategy (\"first\"|\"best\")> <stagnation_limit>";
	auto usage_lns = args[0] + " lns <instance> <stagnation_limit>";

	if (args.size() < 2)
	{
		std::cerr << "Usage:\n"
#ifndef SCFPDP_HEADLESS
			<< "  " << args[0] << " <instance>        (launches GUI)\n"
#endif
			<< "  " << usage_greedy << "\n"
			<< "  " << usage_greedy_rand << "\n"
			<< "  " << usage_random << "\n"
			<< "  " << usage_beam << "\n"
			<< "  " << usage_localsearch << "\n"
			<< "  " << usage_vnd << "\n"
			<< "  " << usage_grasp << "\n"
			<< "  " << usage_lns << "\n";
		return 1;
	}
	else if (args.size() == 2)
	{
#ifdef SCFPDP_HEADLESS
	std::cerr << "GUI unavailable in headless build.\n";
	return 1;
#else
	return gui_application(args);
#endif
	}
	
	auto arg = args.begin() + 1;
	const std::string opt_algo = *(arg++);
	auto instance = Instance::load_from_file((arg++)->c_str());
	auto solution = Solution(&instance);

	Algorithm* algorithm = nullptr;
	if (opt_algo == "greedy")
	{
		// <instance>
		if (args.size() < 3)
		{
			std::cerr << "Usage: " << usage_greedy << "\n";
			return 1;
		}
		algorithm = new GreedyNearestAny(&instance);
	}
	else if (opt_algo == "greedy_rand")
	{
		// <instance> <candidates> <tightening>
		if (args.size() < 5)
		{
			std::cerr << "Usage: " << usage_greedy_rand << "\n";
			return 1;
		}
		auto* greedy_alg = new GreedyRandomizedClosest(&instance);
		greedy_alg->candidates_count = std::stoi(*(arg++));
		greedy_alg->tightening_interval = std::stoi(*(arg++));
		algorithm = greedy_alg;
	}
	else if (opt_algo == "random")
	{
		// <instance>
		if (args.size() < 3)
		{
			std::cerr << "Usage: " << usage_random << "\n";
			return 1;
		}
		algorithm = new RandomConstruction(&instance);
	}
	else if (opt_algo == "beam")
	{
		// <instance> <beam_width>
		if (args.size() < 4)
		{
			std::cerr << "Usage: " << usage_beam << "\n";
			return 1;
		}
		auto* beam_search = new BeamSearch(&instance);
		beam_search->beam_width = std::stoi(*(arg++));
		algorithm = beam_search;
	}
	else if (opt_algo == "localsearch")
	{
		// <instance> <initial_solution> <neighborhood_type> <strategy ("first"|"best")>
		if (args.size() < 6)
		{
			std::cerr << "Usage: " << usage_localsearch << "\n";
			return 1;
		}
		auto* localsearch = new LocalSearch(&instance);
		localsearch->set_initial_solution(Solution::load_from_file((arg++)->c_str(), &instance));
		localsearch->improvement_strategy = *(arg++) == "first"
			? LocalSearch::FIRST_IMPROVEMENT : LocalSearch::BEST_IMPROVEMENT;
		localsearch->neighborhood_type = static_cast<LocalSearch::NeighborhoodType>(std::stoi(*(arg++)));
		algorithm = localsearch;
	}
	else if (opt_algo == "vnd")
	{
		// <instance> <initial_solution>
		if (args.size() < 4)
		{
			std::cerr << "Usage: " << usage_vnd << "\n";
			return 1;
		}
		auto* vnd = new VariableNeighborhoodDescent(&instance);
		vnd->set_initial_solution(Solution::load_from_file((arg++)->c_str(), &instance));
		algorithm = vnd;
	}
	else if (opt_algo == "grasp")
	{
		// <instance> <candidates> <tightening> <neighborhood_type> <strategy ("first"|"best")> <stagnation_limit>
		if (args.size() < 8)
		{
			std::cerr << "Usage: " << usage_grasp << "\n";
			return 1;
		}
		auto construction = std::make_unique<GreedyRandomizedClosest>(&instance);
		construction->candidates_count = std::stoi(*(arg++));
		construction->tightening_interval = std::stoi(*(arg++));
		auto localsearch = std::make_unique<LocalSearch>(&instance);
		localsearch->neighborhood_type = static_cast<LocalSearch::NeighborhoodType>(std::stoi(*(arg++)));
		localsearch->improvement_strategy = *(arg++) == "first"
			? LocalSearch::FIRST_IMPROVEMENT : LocalSearch::BEST_IMPROVEMENT;
		auto* grasp = new Grasp(&instance, std::move(construction), std::move(localsearch));
		grasp->stopping_criterion.type = StoppingCriterion::STAGNATION;
		grasp->stopping_criterion.stagnation_limit = std::stoi(*(arg++));
		algorithm = grasp;
	}
	else if (opt_algo == "lns")
	{
		// <instance> <stagnation_limit>
		if (args.size() < 4)
		{
			std::cerr << "Usage: " << usage_lns << "\n";
			return 1;
		}
		auto* lns = new LargeNeighborhoodSearch(&instance);
		lns->stopping_criterion.type = StoppingCriterion::STAGNATION;
		lns->stopping_criterion.stagnation_limit = std::stoi(*(arg++));
		algorithm = lns;
	}
	else
	{
		std::cerr << "Unknown or incomplete command.\n";
		return 1;
	}

	solution = Algorithm::run(*algorithm);
	solution.print();

	if (arg != args.end())
	{
		std::stringstream comment;
		comment << "solution" << ";" << solution.calculate_cost();
		solution.write_to_file((*arg).c_str(), comment.str().c_str());
		std::cout << "Solution written to " << (*arg) << "\n";
	}

	delete algorithm;
	return 0;
}
