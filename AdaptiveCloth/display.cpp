
#include <Windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>

#include "Application.h"
#include "display.hpp"
#include "bvh.hpp"
#include "geometry.hpp"
#include "io.hpp"
#include "util.hpp"
#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glut.h"
#include "imgui/backends/imgui_impl_opengl2.h"

static std::vector<Mesh *> &meshes = g_App.m_Sim.m_pClothMeshes;
void vertex(const Vec2 &x)
{
    glVertex2d(x[0], x[1]);
}

void vertex(const Vec3 &x)
{
    glVertex3d(x[0], x[1], x[2]);
}

void normal(const Vec3 &n)
{
    glNormal3d(n[0], n[1], n[2]);
}

void color(const Vec3 &x)
{
    glColor3d(x[0], x[1], x[2]);
}

struct View
{
    double lat, lon;
    Vec2 offset;
    double scale;
    View() : lat(0), lon(0), offset(0), scale(0.5) {}
};
View gMainView;

void zoom(bool in)
{
    // int pane = get_pane();
    // if (pane == -1)
    // {
    // 	// ECHO("i don't know what to do with this event"); return;
    // 	pane = 2;
    // }
    View &view = gMainView;
    if (in)
        view.scale *= 1.2;
    else
        view.scale /= 1.2;
    glutPostRedisplay();
}

void reshape(int w, int h)
{
    // int npanes = 0;
    // for (int i = 0; i < 3; i++)
    // 	if (pane_enabled[i])
    // 		npanes++;
    // int j = 0;
    // for (int i = 0; i < 3; i++)
    // {
    // 	glutSetWindow(subwindows[i]);
    // 	int x0 = w * j / npanes, x1 = pane_enabled[i] ? w * (j + 1) / npanes : x0 + 1;

    // 	if (pane_enabled[i])
    // 		j = j + 1;
    // }
    ImGui_ImplGLUT_ReshapeFunc(w, h);

    glutPositionWindow(0, 0);
    glutReshapeWindow(w, h);
    glViewport(0, 0, w, h);
}

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
            hue = -0.6 * M_PI + hue;               // looks better this way :/
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
void glut_display_func()
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
    // apply_view(views[WorldPane]);
    apply_view(gMainView);
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

    // my_display_code();
    ImGui::ShowDemoWindow();

    // Rendering
    ImGui::Render();
    ImGuiIO &io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    // glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context where shaders may be bound, but prefer using the GL3+ code.
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
}

struct MouseState
{
    bool down;
    int x, y;
    enum
    {
        ROTATE,
        TRANSLATE,
        SCALE
    } func;
} mouse_state;

void mouse(int button, int state, int x, int y)
{
    // printf("[mouse] x %d y %d\n", x, y);
    ImGui_ImplGLUT_MouseFunc(button, state, x, y);
    ImGuiIO &io = ImGui::GetIO();
    if (io.WantCaptureMouse)
    {
        return;
    }
    mouse_state.down = (state == GLUT_DOWN);
    mouse_state.x = x;
    mouse_state.y = y;
    // int pane = get_pane();
    // if (pane == -1)
    // {
    // 	ECHO("i don't know what to do with this event");
    // 	return;
    // }
    View &view = gMainView;

    if ((button == 3) || (button == 4))
    { // It's a wheel event
        mouse_state.func = MouseState::SCALE;
        // Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
        if (state == GLUT_UP)
            return; // Disregard redundant GLUT_UP events
        if (button == 3)
        {
            view.scale *= 1.2;
        }
        else
        {
            view.scale /= 1.2;
        }
        glutPostRedisplay();
    }
    else if (button == GLUT_LEFT_BUTTON)
    {
        mouse_state.func = MouseState::ROTATE;
    }
    else if (button == GLUT_MIDDLE_BUTTON)
    {
        mouse_state.func = MouseState::TRANSLATE;
    }
}

template <typename T>
T clamp(const T &val, const T &min, const T &max)
{
    return std::min(std::max(val, min), max);
}
void motion(int x, int y)
{
    // printf("[motion] x %d y %d\n", x, y);
    ImGui_ImplGLUT_MotionFunc(x, y);
    ImGuiIO &io = ImGui::GetIO();
    if (io.WantCaptureMouse == true)
    {
        return;
    }
    if (!mouse_state.down)
        return;
    // int pane = get_pane();
    // if (pane == -1)
    // {
    // 	ECHO("i don't know what to do with this event");
    // 	return;
    // }
    View &view = gMainView;
    if (mouse_state.func == MouseState::ROTATE)
    {
        double speed = 0.25;
        view.lon += (x - mouse_state.x) * speed;
        view.lat += (y - mouse_state.y) * speed;
        view.lat = clamp(view.lat, -90., 90.);
    }
    else if (mouse_state.func == MouseState::TRANSLATE)
    {
        double speed = 1e-3;
        view.offset[0] += (x - mouse_state.x) * speed;
        view.offset[1] -= (y - mouse_state.y) * speed;
    }
    mouse_state.x = x;
    mouse_state.y = y;
    glutPostRedisplay();
}
void run_glut(const GlutCallbacks &cb)
{
    //     int argc = 1;
    //     char argv0[] = "";
    //     char *argv = argv0;
    //     glutInit(&argc, &argv);
    // #ifdef __FREEGLUT_EXT_H__
    //     glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    // #endif

    //     glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
    //     glutInitWindowSize(1280, 720);
    //     int window = glutCreateWindow("ARCSim");
    //     glutDisplayFunc(nop);
    //     glutReshapeFunc(reshape);
    //     glutIdleFunc(cb.idle);
    //     glutKeyboardFunc(cb.keyboard);
    //     glutSpecialFunc(cb.special);
    //     double x[4] = {0, 1280 / 3, 1280 * 2 / 3, 1280};
    //     void (*display[3])() = {display_material, display_plastic, display_world};
    //     for (int i = 0; i < 3; i++)
    //     {
    //         subwindows[i] = glutCreateSubWindow(window, x[i], 0, x[i + 1], 720);
    //         glutDisplayFunc(display[i]);
    //         glutKeyboardFunc(cb.keyboard);
    //         glutSpecialFunc(cb.special);
    //         glutMouseFunc(mouse);
    //         glutMotionFunc(motion);
    //     }
    //     ::pane_enabled[PlasticPane] = g_App.m_Sim.enabled[Simulation::Plasticity];

    //     InitImGUI();
    //     glutMainLoop();

    // ---------------- self code --------------
    int argc = 1;
    char argv0[] = "";
    char *argv = argv0;
    glutInit(&argc, &argv);
#ifdef __FREEGLUT_EXT_H__
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutInitWindowSize(1280, 720);
    glutCreateWindow("ARCSim");

    // set up callbacks
    /*
    glutReshapeFunc
    glutIdleFunc
    glutDisplayFunc
    glutKeyboardFunc
    glutSpecialFunc
    glutMouseFunc
    glutMotionFunc
    */
    glutReshapeFunc(reshape);

    glutIdleFunc(cb.idle);

    glutDisplayFunc(glut_display_func);
    glutKeyboardFunc(cb.keyboard);
    glutSpecialFunc(cb.special);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();

    ImGui::StyleColorsDark();

    ImGui_ImplGLUT_Init();
    // ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();
    glutMainLoop();
}

void redisplay()
{

    // glutSetWindow(subwindows[i]);
    glutPostRedisplay();
    // }
}