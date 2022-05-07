#ifdef _WIN32
#pragma once
#include "utils/JsonUtil.h"
// #include "utils/MathUtil.h"
#include "vectors.hpp"
#include "utils/TimeUtil.hpp"
#include <string>

/**
 * \brief       check whether the simulation has converged to a stable disipline
 */
class cMeshConvergenceChecker
{
public:
    inline static const std::string MAX_CONVERGENCE_ITERS =
                                        "max_convergence_iters",
                                    CHECK_GAP_SECOND = "check_gap_second",
                                    CONV_THRESHOLD = "convergence_threshold_mm",
                                    MIN_CONVERGENCE_ITERS =
                                        "min_convergence_iters";
    explicit cMeshConvergenceChecker();
    void Init(double mCheckgapSec,
              double mConvThresholdmm,
              int mMaxIterationsToConvergence,
              int mMinIterationsToConvergence);
    int GetCheckgapSecond() const;
    int GetMaxIterations() const;
    void SetCheckgapSecond(int);
    double GetConvThresholdmm() const;
    bool NeedToCheck() const;
    bool CheckConvergence(const std::vector<Vec3> &new_mesh, double &diff_mm,
                          int &cur_iters);
    void StartSampling(const std::vector<Vec3> &init_mesh);
    static double CalculateMeshDistmm(const std::vector<Vec3> &mesh0,
                                      const std::vector<Vec3> &mesh1);

protected:
    // --------- variables need to be maintained
    std::vector<Vec3> mMeshBefore; //  the mesh before
    bool mStartSampling;
    cTimePoint mLastTimePoint; // the check time from last check
    int mCurIters;

    // --------- settings
    int mCheckgapSec;                // check gap, second
    double mConvThresholdmm;         // convergence threshold [mm]
    int mMaxIterationsToConvergence; // the upper limit of the convergence
                                     // iterations
    int mMinIterationsToConvergence; // the lower imit of the convergence
                                     // iterations
    void UpdateTimePoint();
    void UpdateMesh(const std::vector<Vec3> &mesh);
};

#endif /*_WIN32*/