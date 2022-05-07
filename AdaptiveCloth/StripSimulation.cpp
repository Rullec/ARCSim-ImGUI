#include "StripSimulation.hpp"
#include "imgui.h"
void StripSimulation::UpdateImGUI()
{
    Simulation::UpdateImGUI();

    ImGui::Text("free length %.3f", GetFreeLength());

    // 1. show cloth AABB
    {
        Vec3 aabb_min, aabb_max;
        aabb_mesh(aabb_min, aabb_max, m_Cloths[0].mesh);
        // ImGui::Text("cloth aabb min %.3f %.3f %.3f", aabb_min[0], aabb_min[1], aabb_min[2]);
        // ImGui::Text("cloth aabb max %.3f %.3f %.3f", aabb_max[0], aabb_max[1], aabb_max[2]);
        Vec3 aabb_size = aabb_max - aabb_min;
        ImGui::Text("cloth aabb size %.3f %.3f %.3f", aabb_size[0], aabb_size[1], aabb_size[2]);
    }
    // 2. show total length
    {
        // ImGui::Text("cloth aabb min %.3f %.3f", aabb_min[0], aabb_min[1]);
        // ImGui::Text("cloth aabb max %.3f %.3f", aabb_max[0], aabb_max[1]);
        Vec2 aabb_size = m_uvmax[0] - m_uvmin[0];
        ImGui::Text("cloth aabb2d size %.3f %.3f", aabb_size[0], aabb_size[1]);
    }
    // 3. show free length

    // set free length
    {
        float new_tar = m_target_freelength;
        ImGui::DragFloat("tar free length", &new_tar, 0.01, 0.01, 0.2);
        if (std::fabs(new_tar - m_target_freelength) > 1e-5)
        {
            m_target_freelength = new_tar;
            SetFreeLength(new_tar);
        }
        ImGui::Text("current handles %d", m_pHandles.size());
    }

    {
        ImGui::Text("HW ratio %.3f", CalculateHWRatio());
    }
}

float StripSimulation::GetFreeLength()
{
    // 1. get constraint
    Vec2 uv_min = m_uvmin[0], uv_max = m_uvmax[0];

    // std::cout << "m_pHandles size = " << m_pHandles.size() << std::endl;
    double u_min = 1e3;
    double u_max = -1e3;
    for (auto &x : m_pHandles)
    {
        auto nodes = x->get_nodes();
        double cur_u = nodes[0]->verts[0]->u[1];
        if (cur_u > u_max)
            u_max = cur_u;
        if (cur_u < u_min)
            u_min = cur_u;
    }
    float fix_length = u_max - u_min;

    return (uv_max - uv_min)[1] - fix_length;
}

void StripSimulation::SetFreeLength(float free_length)
{
    ClothResetInitPos();
    // 1. judge which point should be included
    float total_length = (m_uvmax[0][1] - m_uvmin[0][1]);
    float limit_length = total_length - free_length;
    float v_min = this->m_uvmin[0][1];
    float v_max = v_min + limit_length;

    // 2. clear all handles
    ClearHandles();

    // 3. add new handles
    for (auto node : m_pClothMeshes[0]->nodes)
    {
        float cur_v = node->verts[0]->u[1];
        if (cur_v >= v_min && cur_v <= v_max)
        {
            NodeHandle *han = new NodeHandle;
            han->node = node;
            han->node->preserve = true;
            han->motion = nullptr;
            han->start_time = 0;
            han->end_time = 1e9;
            m_pHandles.push_back(han);
        }
    }
    /*
    NodeHandle *han = new NodeHandle;
    han->node = mesh.nodes[ns[i]];
    han->node->preserve = true;
    han->motion = motion;
    hans.push_back(han);
    */
}
void StripSimulation::ClearHandles()
{
    for (auto &x : this->m_pHandles)
    {
        delete x;
    }
    m_pHandles.clear();
}
void StripSimulation::Prepare()
{
    Simulation::Prepare();
    // begin to calculate the uv range for each cloth
    m_uvmin.clear();
    m_uvmax.clear();
    for (int i = 0; i < m_Cloths.size(); i++)
    {
        Vec2 aabb_min, aabb_max;
        aabb2d_mesh(aabb_min, aabb_max, m_Cloths[i].mesh);
        m_uvmin.push_back(aabb_min);
        m_uvmax.push_back(aabb_max);
    }

    m_target_freelength = GetFreeLength();

    // set init pose
    m_cloth_initpos.resize(this->m_pClothMeshes.size());
    for (int i = 0; i < this->m_pClothMeshes.size(); i++)
    {
        m_cloth_initpos[i].clear();
        for (auto &n : m_pClothMeshes[i]->verts)
        {
            m_cloth_initpos[i].push_back(n->node->x);
        }
    }

    m_target_freelength = 0.02;
    SetFreeLength(m_target_freelength);
}

void StripSimulation::ClothResetInitPos()
{
    for (int i = 0; i < this->m_pClothMeshes.size(); i++)
    {
        auto cur_cloth = m_pClothMeshes[i];
        for (int n_id = 0; n_id < cur_cloth->verts.size(); n_id++)
        {
            // 1. clear velocity, clear x0
            auto cur_v = cur_cloth->verts[n_id];
            cur_v->node->v = Vec3(0, 0, 0);
            cur_v->node->x = this->m_cloth_initpos[i][n_id];
            cur_v->node->x0 = cur_v->node->x;
        }
    }
}
#include <set>
double StripSimulation::CalculateHWRatio()
{
    // 1. get all fixed vertices
    std::set<int> fixed_v_set;
    for (auto &h : m_pHandles)
    {
        fixed_v_set.insert(h->get_nodes()[0]->index);
    }
    // 2. calculate height (Z axis)
    // 3. calculate width (Y axis)
    float z_min = 1e3, z_max = -1e3;
    float y_min = 1e3, y_max = -1e3;
    auto cur_cloth = m_pClothMeshes[0];
    for (int i = 0; i < cur_cloth->verts.size(); i++)
    {
        if (fixed_v_set.find(i) == fixed_v_set.end())
        {
            // not fixed vertex, begin to account for
            auto cur_x = cur_cloth->verts[i]->node->x;
            float y = cur_x[1];
            float z = cur_x[2];
            if (y > y_max)
                y_max = y;
            if (y < y_min)
                y_min = y;
            if (z > z_max)
                z_max = z;
            if (z < z_min)
                z_min = z;
        }
        // auto cur_v = cur_cloth->verts[];
    }

    double width = std::fabs(y_max - y_min);
    double height = std::fabs(z_max - z_min);

    return height / width;
}