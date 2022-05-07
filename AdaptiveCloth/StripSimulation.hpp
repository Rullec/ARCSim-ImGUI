#pragma once
#include "Simulation.hpp"
struct StripSimulation : public Simulation
{
public:
    std::vector<Vec2> m_uvmin;
    std::vector<Vec2> m_uvmax;
    std::vector<std::vector<Vec3>> m_cloth_initpos;
    virtual void UpdateImGUI() override;
    virtual void Prepare() override;

protected:
    float GetFreeLength();
    void SetFreeLength(float val);
    void ClearHandles();
    float m_target_freelength;
    void ClothResetInitPos();
    double CalculateHWRatio();
};