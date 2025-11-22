#pragma once

#include <vector>
#include <variant>

#include "core/primitives.h"
#include "util/gui.h"

struct Color
{
	uint8_t r, g, b;

	static Color random(unsigned seed = 0);
};

static constexpr Color COLOR_BLACK   = {  0,   0,   0};
static constexpr Color COLOR_WHITE   = {255, 255, 255};
static constexpr Color COLOR_LIGHT_GRAY = {192, 192, 192};
static constexpr Color COLOR_GRAY    = {128, 128, 128};
static constexpr Color COLOR_DARK_GRAY = { 64,  64,  64};
static constexpr Color COLOR_RED     = {255,   0,   0};
static constexpr Color COLOR_GREEN   = {  0, 255,   0};
static constexpr Color COLOR_BLUE    = {  0,   0, 255};
static constexpr Color COLOR_CYAN    = {  0, 255, 255};
static constexpr Color COLOR_MAGENTA = {255,   0, 255};
static constexpr Color COLOR_YELLOW  = {255, 255,   0};
static constexpr Color COLOR_ORANGE  = {255, 165,   0};


struct Circle
{
	float x, y;
	float radius;
	Color color;
	bool filled = false;
	Circle() = default;
	Circle(float x, float y, float radius, Color color, bool filled)
		: x(x), y(y), radius(radius), color(color), filled(filled) {}
	Circle(Location center, float radius, Color color, bool filled)
		: x(static_cast<float>(center.x)), y(static_cast<float>(center.y)),
		  radius(radius), color(color), filled(filled) {}
	Circle(const Circle&) = default;
};

struct Line
{
	float x1, y1;
	float x2, y2;
	Color color = COLOR_BLACK;
	Line() = default;
	Line(float x1, float y1, float x2, float y2, Color color)
		: x1(x1), y1(y1), x2(x2), y2(y2), color(color) {}
	Line(Location a, Location b, Color color)
		: x1(static_cast<float>(a.x)), y1(static_cast<float>(a.y)),
		  x2(static_cast<float>(b.x)), y2(static_cast<float>(b.y)),
		  color(color) {}
	Line(const Line&) = default;
};

struct Canvas : public GuiElement
{
	int width = 800;
	int height = 600;
	float scale = 1.0f;
	Color bg_color = COLOR_WHITE;
	float stroke_width = 1.0f;
	using Element = std::variant<Line, Circle>;
	using DrawList = std::vector<Element>;
	DrawList elements = {};
	void clear();
	void add(const Element& element);
	std::pair<int, int> get_max_extent() const;
	void draw() override;
};
