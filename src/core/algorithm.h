#pragma once

#include "core/solution.h"
#include "util/canvas.h"
#include "util/gui.h"

struct Algorithm : public GuiElement
{
	virtual size_t step() = 0;
	virtual void reset() = 0;
	virtual bool done() = 0;

	virtual Solution get_current_solution() = 0;

	void draw() override = 0;
};