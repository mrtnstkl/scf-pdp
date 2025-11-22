#pragma once

#include <vector>
#include <variant>
#include <functional>
#include <GLFW/glfw3.h>
#include "imgui.h"

struct GuiElement
{
	virtual void draw() = 0;
	virtual ~GuiElement() = default;
};

template<typename T>
inline void draw(T& drawable)
{
	// this specialization covers all GuiElement derived types
	drawable.draw();
}

class Gui
{
	struct Window
	{
		ImGuiWindowFlags flags;
		const char* name;
		std::vector<std::variant<GuiElement*, std::function<void()>>> elements;
	};
	Window main_window_;

	GLFWwindow* window_;
	bool should_close_ = false;
	std::vector<Window> windows_;
	void draw_elements();
public:
	bool init(const char* title = "GUI", int width = 800, int height = 800);
	void update();
	void push_element(GuiElement* element);
	void push_element(std::function<void()> element);
	void push_window(const char* name);
	bool should_close() const;
	void close();
};

