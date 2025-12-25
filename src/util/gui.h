#pragma once

#include <vector>
#include <variant>
#include <functional>

struct GuiElement
{
	virtual void draw() = 0;
	virtual ~GuiElement() = default;
};


#ifndef SCFPDP_HEADLESS
#include <GLFW/glfw3.h>
#include "imgui.h"

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

#endif // SCFPDP_HEADLESS