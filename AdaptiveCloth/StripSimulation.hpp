#pragma once
#include "Simulation.hpp"
struct StripSimulation : public Simulation
{
public:
    std::vector<Vec2> m_uvmin;
    std::vector<Vec2> m_uvmax;
    virtual void UpdateImGUI() override;
    virtual void Prepare() override;

protected:
    float GetFreeLength();
    void SetFreeLength(float val);
    void ClearHandles();
    float m_target_freelength;
    double CalculateHWRatio();
};