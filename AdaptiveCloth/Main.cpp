/*************************************************************************
***************************    ARCSim_Main    ****************************
*************************************************************************/

#include "Application.h"
#include "cxxopts.hpp"
#include "utils/LogUtil.h"
#include "utils/FileUtil.h"
#include "utils/JsonUtil.h"
using namespace ARCSim;

/*************************************************************************
*******************************    Main    *******************************
*************************************************************************/

void ParseArg(int argc, char *argv[], std::string &config_path,
              bool &disable_imgui);
void ParseConfig(std::string conf);
int main(int argc, char *argv[])
{
	//	g_App.RunSimulate("Configurations\\flag.json");
	//	g_App.RunSimulate("Configurations\\dress-blue.json");
	// g_App.RunSimulate("Configurations\\dress-yellow.json");
	// g_App.RunSimulate("Configurations\\xudong_drap.json");

    // 1. initialize
	bool disable_imgui = false;
	std::string conf = "";
	ParseArg(argc, argv, conf, disable_imgui);

    // 2. run simulation
	g_App.RunSimulate(conf);
}

void ParseArg(int argc, char *argv[], std::string &config_path,
              bool &disable_imgui)
{
    try
    {
        cxxopts::Options options(argv[0], " - arcsim");
        options.positional_help("[optional args]").show_positional_help();

        options.add_options()("conf", "config path",
                              cxxopts::value<std::string>())(
            "d,disable_imgui", "enable imgui rendering",
            cxxopts::value<bool>()->default_value("false"));

        options.parse_positional({"conf"});
        auto result = options.parse(argc, argv);

        if (result.count("conf"))
        {
            config_path = result["conf"].as<std::string>();
            std::cout << "saw param config = " << config_path << std::endl;
        }
        if (result.count("d"))
        {
            disable_imgui = result["disable_imgui"].as<bool>();
            std::cout << "saw param disable_imgui = " << disable_imgui
                      << std::endl;
        }
    }
    catch (const cxxopts::OptionException &e)
    {
        std::cout << "[error] when parsing, " << e.what() << std::endl;
        exit(1);
    }
    SIM_INFO("conf path {}, enable imgui {}", config_path, disable_imgui);
}


void ParseConfig(std::string conf)
{
    SIM_ASSERT(cFileUtil::ExistsFile(conf) == true);
    Json::Value root;
    cJsonUtil::LoadJson(conf, root);
    // gPause = cJsonUtil::ParseAsBool("pause_at_first", root);
    // gEnableDraw = cJsonUtil::ParseAsBool("enable_draw", root);
    // if (gEnableDraw == true)
    // {
    //     gWindowWidth = cJsonUtil::ParseAsInt("window_width", root);
    //     gWindowHeight = cJsonUtil::ParseAsInt("window_height", root);
    // }
}