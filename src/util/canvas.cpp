#include "canvas.h"

#include <iostream>
#include <cmath>
#include <cstdint>
#include <random>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>

#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif


void Canvas::clear()
{
	elements.clear();
}

void Canvas::add(const std::variant<Line, Circle>& element)
{
	elements.push_back(element);
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


static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void Gui::draw_elements()
{
	static ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings;

	const ImGuiViewport* viewport = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos(viewport->Pos);
	ImGui::SetNextWindowSize(viewport->Size);
	ImGui::SetNextWindowBgAlpha(1.f);
	if (ImGui::Begin("Main Window", nullptr, flags))
	{
		for (GuiElement* element : elements_)
			element->draw();
		elements_.clear();
	}
	ImGui::End();

	static bool show_demo_window = false;
	if (show_demo_window) ImGui::ShowDemoWindow(&show_demo_window);
}

bool Gui::init(const char* title, int width, int height)
{
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
		return false;

	// Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
	// GL ES 2.0 + GLSL 100 (WebGL 1.0)
	const char* glsl_version = "#version 100";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(IMGUI_IMPL_OPENGL_ES3)
	// GL ES 3.0 + GLSL 300 es (WebGL 2.0)
	const char* glsl_version = "#version 300 es";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
	// GL 3.2 + GLSL 150
	const char* glsl_version = "#version 150";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
	// GL 3.0 + GLSL 130
	const char* glsl_version = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	//glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

	// Create window with graphics context
	float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor()); // Valid on GLFW 3.3+ only
	window_ = glfwCreateWindow((int)(width * main_scale), (int)(height * main_scale), title, nullptr, nullptr);
	if (window_ == nullptr)
		return false;
	glfwMakeContextCurrent(window_);
	glfwSwapInterval(1); // Enable vsync

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsLight();

	// Setup scaling
	ImGuiStyle& style = ImGui::GetStyle();
	style.ScaleAllSizes(main_scale);        // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing this requires resetting Style + calling this again)
	style.FontScaleDpi = main_scale;        // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We leave both here for documentation purpose)

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window_, true);
	ImGui_ImplOpenGL3_Init(glsl_version);
	return true;
}

void Gui::update()
{
	if (glfwWindowShouldClose(window_))
	{
		should_close_ = true;
		return;
	}
	glfwPollEvents();
	if (glfwGetWindowAttrib(window_, GLFW_ICONIFIED) != 0)
	{
		ImGui_ImplGlfw_Sleep(10);
		return;
	}

	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	draw_elements();

	// Rendering
	ImGui::Render();
	int display_w, display_h;
	glfwGetFramebufferSize(window_, &display_w, &display_h);
	glViewport(0, 0, display_w, display_h);
	glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
	glClear(GL_COLOR_BUFFER_BIT);
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glfwSwapBuffers(window_);
}

void Gui::push_element(GuiElement* element)
{
	elements_.push_back(element);
}

bool Gui::should_close() const
{
    return should_close_;
}

void Gui::close()
{
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window_);
	glfwTerminate();
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
