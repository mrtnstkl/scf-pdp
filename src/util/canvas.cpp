#include "canvas.h"

#include <iostream>
#include <cmath>
#include <cstdint>
#include <random>

#include "imgui.h"


void Canvas::clear()
{
	elements.clear();
}

void Canvas::add(const Element& element)
{
	elements.push_back(element);
}

std::pair<int, int> Canvas::get_max_extent() const
{
	int max_x = 0;
	int max_y = 0;
	for (const auto& element : elements)
	{
		std::visit([&](auto&& arg) {
			using T = std::decay_t<decltype(arg)>;
			if constexpr (std::is_same_v<T, Line>)
			{
				max_x = std::max({ max_x, static_cast<int>(arg.x1), static_cast<int>(arg.x2) });
				max_y = std::max({ max_y, static_cast<int>(arg.y1), static_cast<int>(arg.y2) });
			}
			else if constexpr (std::is_same_v<T, Circle>)
			{
				max_x = std::max(max_x, static_cast<int>(arg.x + arg.radius));
				max_y = std::max(max_y, static_cast<int>(arg.y + arg.radius));
			}
		}, element);
	}
	return { max_x, max_y };
}

void Canvas::draw()
{
	const ImVec2 p = ImGui::GetCursorScreenPos();
	const auto px = p.x;
	const auto py = p.y;

	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddRectFilled(p, ImVec2(py + width, py + height), ImColor(bg_color.r, bg_color.g, bg_color.b, 255));
	draw_list->AddRect(p, ImVec2(px + width, py + height), ImColor(0, 0, 0, 255), 0.0f, 0, 4.0f);

	const float s = scale;

	const auto draw_line = [&](Line line)
	{
		draw_list->AddLine(
			ImVec2(px + line.x1 * s, py + line.y1 * s),
			ImVec2(px + line.x2 * s, py + line.y2 * s),
			ImColor(line.color.r, line.color.g, line.color.b, 255), stroke_width * s);
	};
	const auto draw_circle = [&](Circle circle)
	{
		if (circle.filled)
			draw_list->AddCircleFilled(
				ImVec2(px + circle.x * s, py + circle.y * s),
				circle.radius * s,
				ImColor(circle.color.r, circle.color.g, circle.color.b, 255), 12);
		else
			draw_list->AddCircle(
				ImVec2(px + circle.x * s, py + circle.y * s),
				circle.radius * s,
				ImColor(circle.color.r, circle.color.g, circle.color.b, 255), 12, stroke_width * s);
	};

	for (auto& element : elements)
	{
		std::visit([&](auto&& arg) {
			using T = std::decay_t<decltype(arg)>;
			if constexpr (std::is_same_v<T, Line>)
				draw_line(arg);
			else if constexpr (std::is_same_v<T, Circle>)
				draw_circle(arg);
		}, element);
	}
	ImGui::Dummy(ImVec2(width, height));
}

Color Color::random(unsigned seed)
{
	// fixed saturation & value for vivid colours
	constexpr float S = 1.0f;
	constexpr float V = 1.0f;

	// random hue in [0, 360)
	static thread_local std::mt19937 rng{std::random_device{}()};
	if (seed != 0)
		rng.seed(seed);
	std::uniform_real_distribution<float> dist(0.0f, 360.0f);
	float H = dist(rng);

	float C = V * S;
	float X = C * (1.0f - std::fabs(std::fmod(H / 60.0f, 2.0f) - 1.0f));
	float m = V - C;

	float r_, g_, b_;

	if      (H < 60)  { r_=C; g_=X; b_=0; }
	else if (H < 120) { r_=X; g_=C; b_=0; }
	else if (H < 180) { r_=0; g_=C; b_=X; }
	else if (H < 240) { r_=0; g_=X; b_=C; }
	else if (H < 300) { r_=X; g_=0; b_=C; }
	else              { r_=C; g_=0; b_=X; }

	return {
		static_cast<uint8_t>((r_ + m) * 255),
		static_cast<uint8_t>((g_ + m) * 255),
		static_cast<uint8_t>((b_ + m) * 255)
	};
}
