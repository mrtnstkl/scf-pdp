#pragma once
#ifndef SCFPDP_HEADLESS

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "core/algorithm.h"
#include "util/canvas.h"
#include "util/gui.h"
#include "util/timer.hpp"

#include "imgui.h"

template <typename A>
class AlgorithmVisualizer : public GuiElement
{
	A* algorithm_;
	size_t position_ = 0;
	size_t max_position_ = 0;
	std::map<size_t, typename A::Metrics> metrics_;
	std::map<size_t, Canvas::DrawList> canvas_states_;
	Canvas current_canvas_;
	enum RecordingLevel
	{
		NONE,
		METRICS,
		CANVAS
	} recording_level_ = CANVAS;
	bool enable_logging_ = true;
	std::pair<std::string, ImVec4> feedback_ = {"", ImVec4(1.f, 1.f, 1.f, 1.f)};

	/**
	 * @brief Record a snapshot of the current solution state
	 *
	 * @param soln the solution whose current state is to be recorded
	 * @param metrics the metrics associated with the current state
	 * @param level the recording level (NONE, METRICS, CANVAS)
	 */
	void record_snapshot(
		const Solution& soln, const typename A::Metrics& metrics, RecordingLevel level);

public:
	AlgorithmVisualizer(A* algorithm);

	void draw() override;
};


class AlgorithmGui : public GuiElement
{
	const char* name_;
	Algorithm* algorithm_;
	std::unique_ptr<GuiElement> visualizer_;

public:
	template <typename A>
	AlgorithmGui(const char* name, A* algo)
		: name_(name), algorithm_(algo), visualizer_(std::make_unique<AlgorithmVisualizer<A>>(algo))
	{
	}
	AlgorithmGui(const AlgorithmGui&) = delete; // cant copy unique_ptr
	AlgorithmGui& operator=(const AlgorithmGui&) = delete;

	void draw() override { algorithm_->draw(); }
	const char* name() const { return name_; }
	GuiElement* get_visualization_gui() { return visualizer_.get(); }
};

template <typename A>
void AlgorithmVisualizer<A>::record_snapshot(
	const Solution& soln, const typename A::Metrics& metrics, RecordingLevel level)
{
	position_ = metrics.step;
	max_position_ = std::max(max_position_, position_);
	if (level >= METRICS)
	{
		metrics_[position_] = metrics;
	}
	if (level >= CANVAS)
	{
		Canvas temp_canvas;
		soln.visualize(temp_canvas);
		canvas_states_[position_] = temp_canvas.elements;
	}
}


template <typename A>
AlgorithmVisualizer<A>::AlgorithmVisualizer(A* algorithm)
	: algorithm_(algorithm)
{
	const float width_target = 560.f;
	algorithm->get_current_solution().visualize(current_canvas_);
	auto [max_x, max_y] = current_canvas_.get_max_extent();
	current_canvas_.scale = width_target / max_x;
	current_canvas_.width = width_target;
	current_canvas_.height = max_y * current_canvas_.scale;
	current_canvas_.bg_color = COLOR_BLACK;
	current_canvas_.stroke_width = .2f;
}


template <typename A>
inline void AlgorithmVisualizer<A>::draw()
{
	// Helper lambdas

	// reset: reset the algorithm and clear recorded data
	const auto reset = [&]()
	{
		position_ = max_position_ = 0;
		algorithm_->reset();
		metrics_.clear();
		canvas_states_.clear();
	};
	// step: advance the algorithm by one step and record data
	const auto step = [&]()
	{
		algorithm_->step();
		record_snapshot(algorithm_->get_current_solution(), algorithm_->metrics, recording_level_);
	};
	// run_to_end: run the algorithm until it is done
	// data is recorded according to the recording level
	// this is safe to run in a separate thread
	const auto run_to_end = [this](const std::atomic_bool& should_stop)
	{
		bool logging = enable_logging_;
		std::ofstream output_file;
		if (logging)
		{
			output_file.open("out/run_log.csv");
			output_file << "time,";
			algorithm_->metrics.print_head(output_file);
			output_file << std::endl;
			logging = output_file.is_open();
		}
		Timer timer;
		record_snapshot(algorithm_->get_current_solution(), algorithm_->metrics, CANVAS);
		if (recording_level_ == NONE)
		{
			while (!algorithm_->done() && !should_stop)
				algorithm_->step();
			record_snapshot(
				algorithm_->get_current_solution(), algorithm_->metrics, recording_level_);
		}
		else
		{
			while (!algorithm_->done() && !should_stop)
			{
				algorithm_->step();
				record_snapshot(algorithm_->get_current_solution(), algorithm_->metrics, recording_level_);
				if (logging)
				{
					output_file << timer.get<std::chrono::milliseconds>() << ','
								<< algorithm_->metrics << std::endl;
				}
			}
		}
		feedback_ = {"Completed in " + timer.get<std::string>(), ImVec4(0.f, 1.f, 0.f, 1.f)};
		// always record final state and display it
		record_snapshot(algorithm_->get_current_solution(), algorithm_->metrics, CANVAS);
		if (!canvas_states_.empty())
			current_canvas_.elements = canvas_states_.rbegin()->second;
		auto& metrics = algorithm_->metrics;
	};


	std::lock_guard<std::mutex> lock(algorithm_->worker.data_mutex);
	if (algorithm_->worker.running)
	{
		ImGui::Text("Algorithm is running...");
		ImGui::Text("Step: %zu", algorithm_->metrics.step);
		ImGui::Text("Cost: %.2f", algorithm_->metrics.total_cost);
		if (ImGui::Button("Request Stop"))
			algorithm_->worker.stop_requested = true;
		if (algorithm_->worker.stop_requested)
			ImGui::Text("Stop requested, please wait.");
		return;
	}

	// Reset and Free Buttons
	// "Reset": resets the algorithm and clears recorded data
	if (ImGui::Button("Reset"))
		reset();
	// "Free": resets and frees up space
	if (ImGui::SameLine(), ImGui::Button("Free"))
	{
		reset();
		metrics_ = {};
		canvas_states_ = {};
	}

	// Step Controls
	ImGui::BeginDisabled(algorithm_->done());
	// "Advance": take one step in the algorithm
	if (ImGui::Button("Advance"))
		step();
	// "Run to End": run the algorithm to completion
	if (ImGui::SameLine(), ImGui::Button("Run to End"))
		algorithm_->worker.start(run_to_end);
	ImGui::EndDisabled();
	// "Re-Run": reset + run to end
	if (ImGui::SameLine(), ImGui::Button("Re-Run"))
	{
		reset();
		algorithm_->worker.start(run_to_end);
		return;
	}

	ImGui::SameLine();
	ImGui::Checkbox("Enable Logging", &enable_logging_);

	// Recording Level
	static const char* levels[] = {"None", "Metrics", "Canvas"};
	static int level_i = 2;
	const char* current_level = (level_i >= 0 && level_i <= 2) ? levels[level_i] : "Unknown";
	if (ImGui::SliderInt("Recording Level", &level_i, 0, 2, current_level))
	{
		level_i = std::clamp(level_i, 0, 2);
		recording_level_ = static_cast<RecordingLevel>(level_i);
	}

	// Log Message
	ImGui::Separator();
	ImGui::TextColored(feedback_.second, "%s", feedback_.first.c_str());
	ImGui::Separator();

	// Step Slider and Display
	ImGui::Text("Current Step: %zu/%zu", position_, max_position_);
	if (ImGui::SliderInt(
			"Step", reinterpret_cast<int*>(&position_), 0, static_cast<int>(max_position_)))
	{
		if (canvas_states_.contains(position_))
			current_canvas_.elements = canvas_states_.at(position_);
	}
	// Canvas and Metrics Display
	if (ImGui::SetNextItemOpen(true, ImGuiCond_Once), ImGui::TreeNode("Graph"))
	{
		current_canvas_.draw();
		ImGui::TreePop();
	}
	if (ImGui::TreeNode("Metrics"))
	{
		if (metrics_.contains(position_))
			metrics_.at(position_).draw();
		else
			ImGui::Text("No metrics recorded for this step.");
		ImGui::TreePop();
	}
	// Cost Chart
	if (ImGui::TreeNode("Metrics - Cost Chart"))
	{
		constexpr auto values_getter = [](void* data, int idx) -> float
		{
			auto& costs = *reinterpret_cast<std::map<size_t, typename A::Metrics>*>(data);
			auto it = costs.find(idx);
			return (it != costs.end()) ? it->second.total_cost : 0.f;
		};
		ImGui::PlotHistogram("Cost over Time", values_getter, reinterpret_cast<void*>(&metrics_),
			static_cast<int>(metrics_.size()), 0, nullptr, FLT_MAX, FLT_MAX, ImVec2(0, 150));
		ImGui::TreePop();
	}

	if (ImGui::Button("Save Last Solution"))
	{
		const auto out_dir = std::filesystem::path("solutions");
		const auto file = (out_dir / "current_solution.sln").string();
		std::filesystem::create_directories(out_dir);
		algorithm_->get_current_solution().write_to_file(file.c_str());
		feedback_ = {"Saved canvas to " + file, ImVec4(0.f, 1.f, 0.f, 1.f)};
	}
}

#endif // SCFPDP_HEADLESS