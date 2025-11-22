#pragma once

#include <map>
#include <iostream>
#include <memory>

#include "util/gui.h"
#include "util/canvas.h"
#include "core/algorithm.h"


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

	void record_snapshot(Solution& soln, typename A::Metrics& metrics)
	{
		position_ = metrics.step;
		max_position_ = std::max(max_position_, position_);
		if (recording_level_ >= METRICS)
		{
			metrics_[position_] = metrics;
		}
		if (recording_level_ >= CANVAS)
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
	if (ImGui::Button("Reset"))
	{
		position_ = max_position_ = 0;
		algorithm_->reset();
		metrics_.clear();
		canvas_states_.clear();
	}
	ImGui::SameLine();
	if (ImGui::Button("Free"))
	{
		position_ = max_position_ = 0;
		algorithm_->reset();
		metrics_ = {};
		canvas_states_ = {};
	}
	ImGui::BeginDisabled(algorithm_->done());
	if (ImGui::Button("Advance"))
	{
		algorithm_->step();
		auto sol = algorithm_->get_current_solution();
		record_snapshot(sol, algorithm_->metrics);
	}
	ImGui::SameLine();
	if (ImGui::Button("Run to End"))
	{
		if (recording_level_ == NONE)
		{
			while (!algorithm_->done()) algorithm_->step();
			auto sol = algorithm_->get_current_solution();
			record_snapshot(sol, algorithm_->metrics);
		}
		else
		{
			while (!algorithm_->done())
			{
				algorithm_->step();
				auto sol = algorithm_->get_current_solution();
				record_snapshot(sol, algorithm_->metrics);
			}
		}
	}
	ImGui::EndDisabled();

	ImGui::Text("Current Step: %zu/%zu", position_, max_position_);
	if (ImGui::SliderInt("Step", reinterpret_cast<int*>(&position_), 0, static_cast<int>(max_position_)))
	{
		if (canvas_states_.contains(position_))
			canvas_.elements = canvas_states_.at(position_);
	}
	canvas_.draw();
	if (metrics_.contains(position_))
		::draw<typename A::Metrics>(metrics_.at(position_));
}


