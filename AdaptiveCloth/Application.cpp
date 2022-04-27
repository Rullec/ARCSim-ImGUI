/*************************************************************************
************************    ARCSim_Application    ************************
*************************************************************************/

#include "conf.hpp"
#include "display.hpp"
#include "Application.h"
#include "separateobs.hpp"

using namespace ARCSim;

Application		g_App;

extern void zoom(bool in);
extern "C" { FILE __iob_func[3] = { *stdin, *stdout, *stderr }; }

/*************************************************************************
***************************    Application    ****************************
*************************************************************************/
Application::Application() : m_IsRunning(false)
{

}


void init_physics(const std::string &json_file)
{
	// std::cout << "begin to load json\n";
	load_json(json_file, g_App.m_Sim);
	// std::cout << "end to load json\n";

	g_App.m_Sim.Prepare();
	
	separate_obstacles(g_App.m_Sim.m_pObstacleMeshes, g_App.m_Sim.m_pClothMeshes);

	g_App.m_Sim.RelaxInitialState();
}


void sim_step()
{
	g_App.m_Sim.AdvanceStep();
}


void idle()
{
	if (!g_App.m_IsRunning)
		return;

	sim_step();
	redisplay();
}


static void keyboard(unsigned char key, int x, int y)
{
	unsigned char esc = 27, space = ' ';

	if (key == esc)
	{
		exit(EXIT_SUCCESS);
	}
	else if (key == space)
	{
		::g_App.m_IsRunning = !::g_App.m_IsRunning;
	}
	else if (key == 's')
	{
		::g_App.m_IsRunning = !::g_App.m_IsRunning;
		idle();
		::g_App.m_IsRunning = !::g_App.m_IsRunning;
	}
	else if (key == 'z')
	{
		zoom(true);
	}
	else if (key == 'x')
	{
		zoom(false);
	}
}


void Application::RunSimulate(std::string jsonPath)
{
	init_physics(jsonPath);

	GlutCallbacks cb;
	cb.idle = idle;
	cb.keyboard = keyboard;

	run_glut(cb);
}


Application::~Application()
{

}