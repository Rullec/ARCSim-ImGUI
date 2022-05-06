/*************************************************************************
************************    ARCSim_Simulation    *************************
*************************************************************************/

#include "magic.hpp"
#include "physics.hpp"
#include "separate.hpp"
#include "collision.hpp"
#include "popfilter.hpp"
#include "proximity.hpp"
#include "simulation.hpp"
#include "plasticity.hpp"
#include "dynamicremesh.hpp"
#include "strainlimiting.hpp"

using namespace std;

Magic magic;
static const bool verbose = false;
static const int proximity = Simulation::Proximity,

physics = Simulation::Physics,
strainlimiting = Simulation::StrainLimiting,
collision = Simulation::Collision,
remeshing = Simulation::Remeshing,
separation = Simulation::Separation,
popfilter = Simulation::PopFilter,
plasticity = Simulation::Plasticity;

/*************************************************************************
****************************    Simulation    ****************************
*************************************************************************/

void Simulation::Prepare()
{
	m_pClothMeshes.resize(m_Cloths.size());

	for (int c = 0; c < m_Cloths.size(); c++)
	{
		m_Cloths[c].ComputeMasses();

		m_pClothMeshes[c] = &m_Cloths[c].mesh;

		update_x0(*m_pClothMeshes[c]);
	}

	m_pObstacleMeshes.resize(m_Obstacles.size());

	for (int o = 0; o < m_Obstacles.size(); o++)
	{
		m_pObstacleMeshes[o] = &m_Obstacles[o].get_mesh();

		update_x0(*m_pObstacleMeshes[o]);
	}
}


void Simulation::RelaxInitialState()
{
	this->ValidateHandles();

	if (::magic.preserve_creases)
	{
		for (int c = 0; c < m_Cloths.size(); c++)
		{
			reset_plasticity(m_Cloths[c]);
		}
	}

	bool equilibrate = true;

	if (::magic.enable_remeshing)
	{
		if (equilibrate)
		{
			this->EquilibrationStep();
			this->RemeshingStep(true);
			this->EquilibrationStep();
		}
		else
		{
			this->RemeshingStep(true);
			this->StrainzeroingStep();
			this->RemeshingStep(true);
			this->StrainzeroingStep();
		}
	}

	if (::magic.preserve_creases)
		for (int c = 0; c < m_Cloths.size(); c++)
			reset_plasticity(m_Cloths[c]);

	::magic.preserve_creases = false;

	if (::magic.fixed_high_res_mesh)
		enabled[remeshing] = false;
}


void Simulation::ValidateHandles()
{
	for (int h = 0; h < m_pHandles.size(); h++)
	{
		vector<Node*> nodes = m_pHandles[h]->get_nodes();

		for (int n = 0; n < nodes.size(); n++)
		{
			if (!nodes[n]->preserve)
			{
				cout << "Constrained node " << nodes[n]->index << " will not be preserved by remeshing" << endl;

				abort();
			}
		}
	}
}


void Simulation::AdvanceStep()
{
	time += step_time;
	step++;

	this->UpdateObstacles(false);

	vector<Constraint*> cons = this->GetConstraints(true);

	this->PhysicsStep(cons);

	this->PlasticityStep();

	this->StrainlimitingStep(cons);

	this->CollisionStep();

	if (step % frame_steps == 0)
	{
		this->RemeshingStep();

		frame++;
	}

	this->DeleteConstraints(cons);

	UpdateImGUI();
}
#include "imgui.h"
void Simulation::UpdateImGUI()
{
	// std::cout << "imgui is updated\n";
    // ImGuiWindowFlags window_flags = 0;
    // // window_flags |= ImGuiWindowFlags_NoMove;
    // // window_flags |= ImGuiWindowFlags_NoResize;
    // bool open = false;
    // bool *p_open = &open;
    // ImGui::Begin("ARCSim", p_open, window_flags);
	// ImGui::Text("Hello, world %d", 123);
    // ImGui::End();
	// ImGui::NewFrame();
}

vector<Constraint*> Simulation::GetConstraints(bool include_proximity)
{
	vector<Constraint*> cons;

	for (int h = 0; h < m_pHandles.size(); h++)
	{
		append(cons, m_pHandles[h]->get_constraints(time));
	}

	if (include_proximity && enabled[proximity])
	{
		append(cons, proximity_constraints(m_pClothMeshes, m_pObstacleMeshes, friction, obs_friction));
	}

	return cons;
}


void Simulation::DeleteConstraints(const vector<Constraint*> &cons)
{
	for (int c = 0; c < cons.size(); c++)
		delete cons[c];
}

// Steps

void update_velocities(vector<Mesh*> &meshes, vector<Vec3> &xold, double dt);



void Simulation::PhysicsStep(const vector<Constraint*> &cons)
{
	if (!enabled[physics])
		return;
	
	for (int c = 0; c < m_Cloths.size(); c++)
	{
		int nn = m_Cloths[c].mesh.nodes.size();

		vector<Vec3>	fext(nn, Vec3(0));
		vector<Mat3x3>	Jext(nn, Mat3x3(0));

		add_external_forces(m_Cloths[c], gravity, wind, fext, Jext);

		for (int m = 0; m < m_Morphs.size(); m++)
			if (m_Morphs[m].mesh == &m_Cloths[c].mesh)
				add_morph_forces(m_Cloths[c], m_Morphs[m], time, step_time, fext, Jext);

		implicit_update(m_Cloths[c], fext, Jext, cons, step_time, false);
	}

	this->StepMesh();
}


void Simulation::StepMesh()
{
	for (int i = 0; i < m_pClothMeshes.size(); i++)
	{
#pragma omp parallel for

		for (int j = 0; j < m_pClothMeshes[i]->nodes.size(); j++)
		{
			m_pClothMeshes[i]->nodes[j]->x += m_pClothMeshes[i]->nodes[j]->v * step_time;
		}
	}


	for (int i = 0; i < m_pObstacleMeshes.size(); i++)
	{
#pragma omp parallel for

		for (int j = 0; j < m_pObstacleMeshes[i]->nodes.size(); j++)
		{
			m_pObstacleMeshes[i]->nodes[j]->x += m_pObstacleMeshes[i]->nodes[j]->v * step_time;
		}
	}
}


void Simulation::PlasticityStep()
{
	if (!enabled[plasticity])
		return;

	for (int c = 0; c < m_Cloths.size(); c++)
	{
		plastic_update(m_Cloths[c]);

		optimize_plastic_embedding(m_Cloths[c]);
	}
}


void Simulation::StrainlimitingStep(const vector<Constraint*> &cons)
{
	if (!enabled[strainlimiting])
		return;
	
	vector<Vec3> xold = node_positions(m_pClothMeshes);

	strain_limiting(m_pClothMeshes, get_strain_limits(m_Cloths), cons);

	update_velocities(m_pClothMeshes, xold, step_time);
}


void Simulation::EquilibrationStep()
{
	vector<Constraint*> cons;// = get_constraints(sim, true);
	// double stiff = 1;
	// swap(stiff, ::magic.handle_stiffness);
	for (int c = 0; c < m_Cloths.size(); c++)
	{
		Mesh &mesh = m_Cloths[c].mesh;
		for (int n = 0; n < mesh.nodes.size(); n++)
			mesh.nodes[n]->acceleration = Vec3(0);
		apply_pop_filter(m_Cloths[c], cons, 1);
	}
	// swap(stiff, ::magic.handle_stiffness);
	
	DeleteConstraints(cons);

	cons = GetConstraints(false);

	if (enabled[collision])
	{
		collision_response(m_pClothMeshes, cons, m_pObstacleMeshes);
	}

	DeleteConstraints(cons);
}


void Simulation::StrainzeroingStep()
{
	vector<Vec2> strain_limits(size<Face>(m_pClothMeshes), Vec2(1, 1));

	vector<Constraint*> cons = proximity_constraints(m_pClothMeshes, m_pObstacleMeshes, friction, obs_friction);

	strain_limiting(m_pClothMeshes, strain_limits, cons);

	DeleteConstraints(cons);
	
	if (enabled[collision])
	{
		collision_response(m_pClothMeshes, vector<Constraint*>(), m_pObstacleMeshes);
	}
}


void Simulation::CollisionStep()
{
	if (!enabled[collision])
		return;

	vector<Vec3> xold = node_positions(m_pClothMeshes);
	vector<Constraint*> cons = GetConstraints(false);
	collision_response(m_pClothMeshes, cons, m_pObstacleMeshes);
	DeleteConstraints(cons);
	update_velocities(m_pClothMeshes, xold, step_time);
}


void Simulation::RemeshingStep(bool initializing)
{
	if (!enabled[remeshing])
		return;

	// copy old meshes
	vector<Mesh> old_meshes(m_Cloths.size());
	vector<Mesh*> old_meshes_p(m_Cloths.size()); // for symmetry in separate()

	for (int c = 0; c < m_Cloths.size(); c++)
	{
		old_meshes[c] = deep_copy(m_Cloths[c].mesh);
		old_meshes_p[c] = &old_meshes[c];
	}
	// back up residuals
	typedef vector<Residual> MeshResidual;
	vector<MeshResidual> res;

	if (enabled[plasticity] && !initializing)
	{
		res.resize(m_Cloths.size());

		for (int c = 0; c < m_Cloths.size(); c++)
			res[c] = back_up_residuals(m_Cloths[c].mesh);
	}
	// remesh
	
	for (int c = 0; c < m_Cloths.size(); c++)
	{
		if (::magic.fixed_high_res_mesh)
			static_remesh(m_Cloths[c]);
		else
		{
			vector<Plane> planes = nearest_obstacle_planes(m_Cloths[c].mesh,
														   m_pObstacleMeshes);
			dynamic_remesh(m_Cloths[c], planes, enabled[plasticity]);
		}
	}
	
	// restore residuals
	if (enabled[plasticity] && !initializing)
	{
		for (int c = 0; c < m_Cloths.size(); c++)
			restore_residuals(m_Cloths[c].mesh, old_meshes[c], res[c]);
	}
	// separate
	if (enabled[separation])
	{
		separate(m_pClothMeshes, old_meshes_p, m_pObstacleMeshes);
	}
	// apply pop filter
	if (enabled[popfilter] && !initializing)
	{
		vector<Constraint*> cons = GetConstraints(true);

		for (int c = 0; c < m_Cloths.size(); c++)
			apply_pop_filter(m_Cloths[c], cons);

		DeleteConstraints(cons);
	}
	// delete old meshes
	for (int c = 0; c < m_Cloths.size(); c++)
		delete_mesh(old_meshes[c]);
}


void update_velocities(vector<Mesh*> &meshes, vector<Vec3> &xold, double dt)
{
	double inv_dt = 1 / dt;

#pragma omp parallel for

	for (int n = 0; n < xold.size(); n++)
	{
		Node *node = get<Node>(n, meshes);

		node->v += (node->x - xold[n]) * inv_dt;
	}
}


void Simulation::UpdateObstacles(bool update_positions)
{
	double decay_time = 0.1,
		blend = step_time / decay_time;

	blend = blend / (1 + blend);

	for (int o = 0; o < m_Obstacles.size(); o++)
	{
		m_Obstacles[o].get_mesh(time);

		m_Obstacles[o].blend_with_previous(time, step_time, blend);

		if (!update_positions)
		{
			// put positions back where they were
			Mesh &mesh = m_Obstacles[o].get_mesh();

			for (int n = 0; n < mesh.nodes.size(); n++)
			{
				Node *node = mesh.nodes[n];

				node->v = (node->x - node->x0) / step_time;

				node->x = node->x0;
			}
		}
	}
}

// Helper functions

template <typename Prim> int size(const vector<Mesh*> &meshes)
{
	int np = 0;
	for (int m = 0; m < meshes.size(); m++) np += get<Prim>(*meshes[m]).size();
	return np;
}

template int size<Vert>(const vector<Mesh*>&);
template int size<Node>(const vector<Mesh*>&);
template int size<Edge>(const vector<Mesh*>&);
template int size<Face>(const vector<Mesh*>&);

template <typename Prim> int get_index(const Prim *p, const vector<Mesh*> &meshes)
{
	int i = 0;
	for (int m = 0; m < meshes.size(); m++)
	{
		const vector<Prim*> &ps = get<Prim>(*meshes[m]);
		if (p->index < ps.size() && p == ps[p->index])
			return i + p->index;
		else i += ps.size();
	}
	return -1;
}
template int get_index(const Vert*, const vector<Mesh*>&);
template int get_index(const Node*, const vector<Mesh*>&);
template int get_index(const Edge*, const vector<Mesh*>&);
template int get_index(const Face*, const vector<Mesh*>&);

template <typename Prim> Prim *get(int i, const vector<Mesh*> &meshes)
{
	for (int m = 0; m < meshes.size(); m++)
	{
		const vector<Prim*> &ps = get<Prim>(*meshes[m]);
		if (i < ps.size())
			return ps[i];
		else
			i -= ps.size();
	}
	return NULL;
}

template Vert *get(int, const vector<Mesh*>&);
template Node *get(int, const vector<Mesh*>&);
template Edge *get(int, const vector<Mesh*>&);
template Face *get(int, const vector<Mesh*>&);

vector<Vec3> node_positions(const vector<Mesh*> &meshes)
{
	vector<Vec3> xs(size<Node>(meshes));

	for (int n = 0; n < xs.size(); n++)
		xs[n] = get<Node>(n, meshes)->x;

	return xs;
}