/*************************************************************************
************************    ARCSim_Application    ************************
*************************************************************************/
#pragma once

#include "Simulation.hpp"

namespace ARCSim
{
	/*********************************************************************
	*************************    Application    **************************
	*********************************************************************/

	class Application
	{

	public:

		Application();
		~Application();

	public:

		void RunSimulate(std::string jsonPath);

	private:



	public:

		Simulation				m_Sim;
		volatile bool			m_IsRunning;
	};
}

extern ARCSim::Application		g_App;