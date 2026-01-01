#pragma once

#include <vector>
#include <functional>
#include <memory>

#include "core/algorithm.h"
#include "core/instance.h"
#include "core/solution.h"
#include "util/gui.h"

namespace Lns
{
	struct PartialSolution
	{
		Solution sol;
		std::vector<CustomerRequest*> unhandled_requests;
		DeltaCalculator delta;
		PartialSolution(const Solution& solution);
	};

	struct SolutionModifier : public GuiElement
	{
		virtual void operator()(PartialSolution& ps) = 0;
		virtual void draw() override { }
	};

	struct DestroyRandomRequests : public SolutionModifier
	{
		float fraction_to_remove = 0.1f;
		void operator()(PartialSolution& ps) override;
		void draw() override;
	};

	struct RepairGreedyInsertion : public SolutionModifier
	{
		void operator()(PartialSolution& ps) override;
		void draw() override;
	};
};

class LargeNeighborhoodSearch : public Algorithm
{
public:
	enum ConstructionMethod : int
	{
		FILE,
		GREEDY_NEAREST_ANY, GREEDY_RANDOMIZED_CLOSEST, RANDOM_CONSTRUCTION,
	};

private:
	struct ModifierFunction
	{
		std::unique_ptr<Lns::SolutionModifier> func;
		float weight = 1.f;
		uint64_t applications = 0;
		uint64_t sucesses = 0;
		void reset();
		ModifierFunction(std::unique_ptr<Lns::SolutionModifier> f);
	};

	Instance* instance_;
	Lns::PartialSolution current_solution_;
	std::vector<ModifierFunction> destroy_functions_;
	std::vector<ModifierFunction> repair_functions_;
	bool finished_ = false;
	
	/**
	 * @brief Select a modifier function using roulette wheel selection
	 * 
	 * @param functions The list of modifier functions with their associated weights
	 * @return ModifierFunction& The winning modifier function
	 */
	static ModifierFunction& roulette_select(std::vector<ModifierFunction>& functions);
	
public:
	using Metrics = SolutionMetrics;
	Metrics metrics;

	float reaction_factor = 0.1f;
	StoppingCriterion stopping_criterion;
	ConstructionMethod construction_method = RANDOM_CONSTRUCTION;

	LargeNeighborhoodSearch(Instance* instance);

	size_t step() override;
	void reset() override;
	bool done() override;

	Solution get_current_solution() override;

	void draw() override;
};