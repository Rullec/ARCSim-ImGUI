#include "ConvergenceChecker.h"
#include <iostream>

cMeshConvergenceChecker::cMeshConvergenceChecker()
{
    mCurIters = 0;
    mStartSampling = false;
}
void cMeshConvergenceChecker::Init(double mCheckgapSec,
                                   double mConvThresholdmm,
                                   int mMaxIterationsToConvergence,
                                   int mMinIterationsToConvergence)
{
    // mCheckgapSec = cJsonUtil::ParseAsInt(CHECK_GAP_SECOND, conf_json);
    // mConvThresholdmm = cJsonUtil::ParseAsDouble(CONV_THRESHOLD, conf_json);
    // mMaxIterationsToConvergence =
    //     cJsonUtil::ParseAsInt(MAX_CONVERGENCE_ITERS, conf_json);
    // mMinIterationsToConvergence =
    //     cJsonUtil::ParseAsInt(MIN_CONVERGENCE_ITERS, conf_json);
    SIM_ASSERT(mMinIterationsToConvergence < mMaxIterationsToConvergence);
}

bool cMeshConvergenceChecker::NeedToCheck() const
{
    SIM_ASSERT(mStartSampling == true);
    // 1. get current time

    double time_elasped_s =
        cTimeUtil::CalcTimeElaspedms(mLastTimePoint,
                                     cTimeUtil::GetCurrentTime_chrono()) *
        1e-3;
    // std::cout << "[need to check] current time elasped = " << time_elasped_s
    //           << " check gap sec = " << mCheckgapSec << std::endl;
    return time_elasped_s > mCheckgapSec;
}

bool cMeshConvergenceChecker::CheckConvergence(const std::vector<Vec3> &new_mesh,
                                               double &diff_mm, int &cur_iters)
{
    mCurIters += 1;
    cur_iters = mCurIters;
    bool ret = false;
    diff_mm = CalculateMeshDistmm(mMeshBefore, new_mesh);
    if (mCurIters < mMinIterationsToConvergence)
    {
        ret = false;
    }
    else if (mCurIters >= mMaxIterationsToConvergence)
    {
        // we cost at most (gap * max) s
        SIM_WARN("max iterations {} exceed; current mesh diff {} mm, the "
                 "standard is {} mm",
                 mMaxIterationsToConvergence, diff_mm, mConvThresholdmm);
        ret = true;
    }
    else
    {
        if (diff_mm < this->mConvThresholdmm)
        {
            // std::cout << "[check_convergence] cur diff mm = " << diff_mm
            //           << " , threshold = " << mConvThresholdmm
            //           << ", converged!\n";
            ret = true;
        }
        else
        {
            // std::cout << "[check_convergence] cur diff mm = " << diff_mm
            //           << " , threshold = " << mConvThresholdmm << std::endl;
            ret = false;
        }
    }

    UpdateTimePoint();
    UpdateMesh(new_mesh);
    return ret;
}

void cMeshConvergenceChecker::StartSampling(const std::vector<Vec3> &init_mesh)
{
    mCurIters = 0;
    mStartSampling = true;
    UpdateTimePoint();
    UpdateMesh(init_mesh);
}

int cMeshConvergenceChecker::GetCheckgapSecond() const { return mCheckgapSec; }

int cMeshConvergenceChecker::GetMaxIterations() const
{
    return mMaxIterationsToConvergence;
}
void cMeshConvergenceChecker::SetCheckgapSecond(int val) { mCheckgapSec = val; }

void cMeshConvergenceChecker::UpdateTimePoint()
{
    mLastTimePoint = cTimeUtil::GetCurrentTime_chrono();
}

void cMeshConvergenceChecker::UpdateMesh(const std::vector<Vec3> &mesh)
{
    mMeshBefore.resize(mesh.size());
    for (int i = 0; i < mesh.size(); i++)
    {
        mMeshBefore[i] = mesh[i];
    }
    // mMeshBefore.noalias() = mesh;
}

double cMeshConvergenceChecker::CalculateMeshDistmm(const std::vector<Vec3> &v0,
                                                    const std::vector<Vec3> &v1)
{
    // std::cout << "begin to compare v0 and v1\n";
    // std::cout << v0.size() << std::endl;
    // std::cout << v1.size() << std::endl;
    double max_move_dist_mm = -1;
    for (int i = 0; i < v0.size(); i++)
    {
        Vec3 p0 = v0[i];
        Vec3 p1 = v1[i];
        double diff_norm_mm = norm(p0 - p1) * 1e3;
        max_move_dist_mm = std::max(max_move_dist_mm, diff_norm_mm);
    }
    return max_move_dist_mm;
    //     int num_of_points = int(v0.size() / 3);
    //     tVectorXd diff_vec = v0 - v1;
    //     // std::cout << "diff_vec = " << diff_vec.transpose() << std::endl;
    //     Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>
    //         diff_mat(diff_vec.data(), num_of_points, 3);
    //     tVectorXd rowwise_norm = diff_mat.rowwise().norm();
    //     double max_move_dist_mm = rowwise_norm.maxCoeff() * 1e3;
    // std:;
    // std::cout << "diff = " << rowwise_norm.transpose() << std::endl;
    // exit(1);
    // return max_move_dist_mm;
}

double cMeshConvergenceChecker::GetConvThresholdmm() const
{
    return mConvThresholdmm;
}