
#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glut.h"
#include "imgui/backends/imgui_impl_opengl2.h"
#ifdef __APPLE__
    #include <GLUT/glut.h>
#else
    #include <GL/freeglut.h>
#endif

#ifdef _MSC_VER
#pragma warning (disable: 4505) // unreferenced local function has been removed
#endif
#include <iostream>
#include "vectors.hpp"

// Our state
static bool show_demo_window = true;
static bool show_another_window = false;
static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

#include "AdaptiveCloth/geometry.hpp"

struct View
{
	double lat, lon;
	Vec2 offset;
	double scale;
	View() : lat(0), lon(0), offset(0), scale(0.5) {}
};
View gWorldView;

template <Space s>
void draw_mesh(const Mesh &mesh, bool set_color = false)
{
	if (set_color)
		glDisable(GL_COLOR_MATERIAL);
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < mesh.faces.size(); i++)
	{
		Face *face = mesh.faces[i];
		if (i % 256 == 0)
		{
			glEnd();
			glBegin(GL_TRIANGLES);
		}
		if (set_color)
		{
			int c = find((Mesh *)&mesh, ::meshes);
			static const float phi = (1 + sqrt(5)) / 2;
			double hue = c * (2 - phi) * 2 * M_PI; // golden angle
			hue = -0.6 * M_PI + hue;			   // looks better this way :/
			if (face->label % 2 == 1)
				hue += M_PI;
			static Vec3 a = Vec3(0.92, -0.39, 0), b = Vec3(0.05, 0.12, -0.99);
			Vec3 frt = Vec3(0.7, 0.7, 0.7) + (a * cos(hue) + b * sin(hue)) * 0.3,
				 bak = frt * 0.5 + Vec3(0.5, 0.5, 0.5);
			float front[4] = {frt[0], frt[1], frt[2], 1},
				  back[4] = {bak[0], bak[1], bak[2], 1};
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, front);
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, back);
			// color(area_color(face));
		}
		normal(nor<s>(face));
		for (int v = 0; v < 3; v++)
			vertex(pos<s>(face->v[v]->node));
	}
	glEnd();
	if (set_color)
		glEnable(GL_COLOR_MATERIAL);
}

template <Space s>
void draw_meshes(bool set_color = false)
{
	for (int m = 0; m < meshes.size(); m++)
		draw_mesh<s>(*meshes[m], set_color);
}


template <Space s>
void draw_seam_or_boundary_edges()
{
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);
	for (int m = 0; m < meshes.size(); m++)
	{
		const Mesh &mesh = *meshes[m];
		for (int e = 0; e < mesh.edges.size(); e++)
		{
			const Edge *edge = mesh.edges[e];
			if (!is_seam_or_boundary(edge))
				continue;
			vertex(pos<s>(edge->n[0]));
			vertex(pos<s>(edge->n[1]));
		}
	}
	glEnd();
}
void my_display_code()
{
    // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
    if (show_demo_window)
        ImGui::ShowDemoWindow(&show_demo_window);

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
    {
        static float f = 0.0f;
        static int counter = 0;

        ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

        ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
        ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
        ImGui::Checkbox("Another Window", &show_another_window);

        ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
        ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

        if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
            counter++;
        ImGui::SameLine();
        ImGui::Text("counter = %d", counter);

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }

    // 3. Show another simple window.
    if (show_another_window)
    {
        ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
        ImGui::Text("Hello from another window!");
        if (ImGui::Button("Close Me"))
            show_another_window = false;
        ImGui::End();
    }
}

void glut_display_func()
{
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    my_display_code();

    // Rendering
    ImGui::Render();
    ImGuiIO& io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    //glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context where shaders may be bound, but prefer using the GL3+ code.
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
}

// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.


double aspect_ratio()
{
	return (double)glutGet(GLUT_WINDOW_WIDTH) / glutGet(GLUT_WINDOW_HEIGHT);
}

void directional_light(int i, const Vec3 &dir, const Vec3 &dif)
{
	float diffuse[4] = {dif[0], dif[1], dif[2], 1};
	float position[4] = {dir[0], dir[1], dir[2], 0};
	glEnable(GL_LIGHT0 + i);
	glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0 + i, GL_POSITION, position);
}

void ambient_light(const Vec3 &a)
{
	float ambient[4] = {a[0], a[1], a[2], 1};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
}

void apply_view(const View &view, bool rotate = true)
{
	glTranslatef(view.offset[0], view.offset[1], 0);
	glScalef(view.scale, view.scale, view.scale);
	if (rotate)
	{
		glRotatef(view.lat - 90, 1, 0, 0);
		glRotatef(view.lon, 0, 0, 1);
	}
}
void display_world()
{
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplGLUT_NewFrame();

	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1, 1);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, aspect_ratio(), 0.1, 10);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0, 0, -1);
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	directional_light(0, Vec3(0, 0, 1), Vec3(0.5, 0.5, 0.5));
	ambient_light(Vec3(0.5));
	apply_view(gWorldView);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	draw_meshes<WS>(true);
	glEnable(GL_CULL_FACE);
	glColor3f(0.8, 0.8, 0.8);
	for (int o = 0; o < g_App.m_Sim.m_Obstacles.size(); o++)
		draw_mesh<WS>(g_App.m_Sim.m_Obstacles[o].get_mesh());
	glDisable(GL_CULL_FACE);
	glColor4d(0, 0, 0, 0.2);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	draw_meshes<WS>();
	draw_seam_or_boundary_edges<WS>();

	ImGui::Render();
	ImGuiIO &io = ImGui::GetIO();
	glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT);
	// glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context where shaders may be bound, but prefer using the GL3+ code.
	ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

	glutSwapBuffers();
	GLenum errCode;
	const GLubyte *errString;
	if ((errCode = glGetError()) != GL_NO_ERROR)
	{
		errString = gluErrorString(errCode);
		std::cout << "OpenGL Error: " << errString << std::endl;
	}
}


int main(int argc, char** argv)
{
    // Create GLUT window
    glutInit(&argc, argv);
#ifdef __FREEGLUT_EXT_H__
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_MULTISAMPLE);
    glutInitWindowSize(1280, 720);
    glutCreateWindow("Dear ImGui GLUT+OpenGL2 Example");

    // Setup GLUT display function
    // We will also call ImGui_ImplGLUT_InstallFuncs() to get all the other functions installed for us,
    // otherwise it is possible to install our own functions and call the imgui_impl_glut.h functions ourselves.
    glutDisplayFunc(glut_display_func);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    // FIXME: Consider reworking this example to install our own GLUT funcs + forward calls ImGui_ImplGLUT_XXX ones, instead of using ImGui_ImplGLUT_InstallFuncs().
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != NULL);

    glutMainLoop();

    // Cleanup
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();
    ImGui::DestroyContext();

    return 0;
}