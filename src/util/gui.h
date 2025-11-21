#pragma once

#include <vector>
#include <GLFW/glfw3.h>

struct GuiElement
{
	virtual void draw() = 0;
};

class Gui
{
	GLFWwindow* window_;
	bool should_close_ = false;
	std::vector<GuiElement*> elements_;
	void draw_elements();
public:
	bool init(const char* title = "GUI", int width = 800, int height = 800);
	void update();
	void push_element(GuiElement* element);
	bool should_close() const;
	void close();
};

