#pragma once

#include <map>
#include <iostream>
#include <memory>

#include "core/algorithm.h"
#include "util/gui.h"
#include "util/canvas.h"
#include "util/timer.hpp"

#include "imgui.h"

template<typename A>
class AlgorithmVisualizer : public GuiElement 
{
	A* algorithm_;
	size_t position_ = 0;
	size_t max_position_ = 0;
	std::map<size_t, typename A::Metrics> metrics_;
	std::map<size_t, Canvas::DrawList> canvas_states_;
	Canvas canvas_;
	enum RecordingLevel { NONE, METRICS, CANVAS } recording_level_ = CANVAS;
	std::pair<std::string, ImVec4> feedback_ = {"", ImVec4(1.f, 1.f, 1.f, 1.f)};

	void record_snapshot(const Solution& soln, typename A::Metrics& metrics, RecordingLevel level)
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
			soln.draw(temp_canvas);
			canvas_states_[position_] = temp_canvas.elements;
		}
	}

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
	template<typename A>
	AlgorithmGui(const char* name, A* algo)
		: name_(name), algorithm_(algo), visualizer_(std::make_unique<AlgorithmVisualizer<A>>(algo)) {}
	AlgorithmGui(const AlgorithmGui&) = delete; // cant copy unique_ptr
	AlgorithmGui& operator=(const AlgorithmGui&) = delete;

	void draw() override { algorithm_->draw(); }
	const char* name() const { return name_; }
	GuiElement* get_visualization_gui() { return visualizer_.get(); }
};



template<typename A>
AlgorithmVisualizer<A>::AlgorithmVisualizer(A* algorithm)
	: algorithm_(algorithm)
{
	const float width_target = 560.f;
	algorithm->get_current_solution().draw(canvas_);
	auto [max_x, max_y] = canvas_.get_max_extent();
	canvas_.scale = width_target / max_x;
	canvas_.width = width_target;
	canvas_.height = max_y * canvas_.scale;
	canvas_.bg_color = COLOR_BLACK;
	canvas_.stroke_width = .2f;
}

template<typename A>
inline void AlgorithmVisualizer<A>::draw()
{
	const auto reset = [&]() {
		position_ = max_position_ = 0;
		algorithm_->reset();
		metrics_.clear();
		canvas_states_.clear();
	};
	const auto step = [&]() {
		algorithm_->step();
		record_snapshot(algorithm_->get_current_solution(), algorithm_->metrics, recording_level_);
	};
	const auto run_to_end = [&]() {
		Timer timer;
		if (recording_level_ == NONE)
		{
			while (!algorithm_->done()) algorithm_->step();
			record_snapshot(algorithm_->get_current_solution(), algorithm_->metrics, recording_level_);
		}
		else
		{
			while (!algorithm_->done())
				step();
		}
		feedback_ = { "Completed in " + timer.get<std::string>(), ImVec4(0.f, 1.f, 0.f, 1.f) };
	};
	const auto display = [&]() {
		record_snapshot(algorithm_->get_current_solution(), algorithm_->metrics, CANVAS);
		if (canvas_states_.contains(position_))
			canvas_.elements = canvas_states_.at(position_);
	};


	// Reset and Free Buttons
	if (ImGui::Button("Reset"))
		reset();
	ImGui::SameLine();
	if (ImGui::Button("Free"))
		{ reset(); metrics_ = {}; canvas_states_ = {}; }

	// Step Controls
	ImGui::BeginDisabled(algorithm_->done());
	if (ImGui::Button("Advance"))
	{
		step();
		display();
	}
	ImGui::SameLine();
	if (ImGui::Button("Run to End"))
	{
		run_to_end();
		display();
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button("Re-Run"))
	{
		reset();
		run_to_end();
		display();
	}
	
	// Recording Level
	static const char* levels[] = { "None", "Metrics", "Canvas" };
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
	if (ImGui::SliderInt("Step", reinterpret_cast<int*>(&position_), 0, static_cast<int>(max_position_)))
	{
		if (canvas_states_.contains(position_))
		canvas_.elements = canvas_states_.at(position_);
	}
	// Canvas and Metrics Display
	if (ImGui::SetNextItemOpen(true, ImGuiCond_Once), ImGui::TreeNode("Graph"))
	{
		canvas_.draw();
		ImGui::TreePop();
	}
	if (ImGui::TreeNode("Metrics"))
	{
		if (metrics_.contains(position_))
			::draw<typename A::Metrics>(metrics_.at(position_));
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
		ImGui::PlotHistogram("Cost over Time", values_getter,
			reinterpret_cast<void*>(&metrics_), static_cast<int>(metrics_.size()),
			0, nullptr, FLT_MAX, FLT_MAX, ImVec2(0,150));
		ImGui::TreePop();
	}

	// Save Button
	if (ImGui::Button("Save Current Canvas"))
	{
		std::string filename = std::string("solutions/current_solution") + ".sln";
		algorithm_->get_current_solution().write_to_file(filename.c_str());
		feedback_ = { "Saved canvas to " + filename, ImVec4(0.f, 1.f, 0.f, 1.f) };
	}
}


