// #include "StripSynthesis.hpp"
// #include "ConvergenceChecker.h"

// void StripSynthesis::UpdateImGUI()
// {
// }
// void StripSynthesis::Prepare()
// {
//     StripSimulation::Prepare();
//     mChecker = new cMeshConvergenceChecker();
//     GenerateProps();
// }

// void StripSynthesis::AdvanceStep()
// {
//     Simulation::AdvanceStep();

//     // 1. check convergence
//     // 1.1 if convergence; dump the cloth & info, jump to next property;
//     // 1.2 if not convergence. continue to judge

//     // {

//     // }
// }

// void StripSynthesis::GenerateProps()
// {
//     mProps.clear();

//     // 0.02 m
//     double st = 100, ed = 2e4;
//     {
//         double log_st = std::log10(st), log_ed = std::log10(ed);
//         int num_of_div = 100;
//         for (int i = 0; i < 100; i++)
//         {
//             double prop = std::pow(10, log_st + (log_ed - log_st) / num_of_div * i);
//             auto cur_prop = tStripProp(0.02, prop);
//             std::cout << "cur prop =  " << cur_prop << std::endl;
//             mProps.push_back(cur_prop);
//         }
//     }

//     // 0.04 m
//     {
//         st = 500, ed = 7e4;

//         double log_st = std::log10(st), log_ed = std::log10(ed);
//         int num_of_div = 100;
//         for (int i = 0; i < 100; i++)
//         {
//             double prop = std::pow(10, log_st + (log_ed - log_st) / num_of_div * i);
//             auto cur_prop = tStripProp(0.02, prop);
//             std::cout << "cur prop =  " << cur_prop << std::endl;
//             mProps.push_back(cur_prop);
//         }
//     }
//     exit(1);
// }