/*************************************************************************
************************    ARCSim_Simulation    *************************
*************************************************************************/
#pragma once

#include "cloth.hpp"
#include "morph.hpp"
#include "handle.hpp"
#include "obstacle.hpp"
#include "constraint.hpp"

/*************************************************************************
****************************    Simulation    ****************************
*************************************************************************/

struct Wind
{
	double density;
	Vec3 velocity;
	double drag;
};

struct Simulation
{
	// variables
	double time;
	int frame, step;

	// constants
	int frame_steps;
	double frame_time, step_time;
	double end_time, end_frame;
	Vec3 gravity;
	Wind wind;
	double friction, obs_friction;

	enum
	{
		Proximity,
		Physics,
		StrainLimiting,
		Collision,
		Remeshing,
		Separation,
		PopFilter,
		Plasticity,
		nModules
	};
	bool enabled[nModules];

	std::vector<Morph> m_Morphs;
	std::vector<Cloth> m_Cloths;
	std::vector<Motion> m_Motions;
	std::vector<Handle *> m_pHandles;
	std::vector<Obstacle> m_Obstacles;
	std::vector<Constraint *> m_pConstraits;
	std::vector<Mesh *> m_pClothMeshes;
	std::vector<Mesh *> m_pObstacleMeshes;

public:
	virtual void Prepare();
	virtual void AdvanceStep();
	void RelaxInitialState();
	virtual void UpdateImGUI();

private:
	void StepMesh();
	void CollisionStep();
	void PlasticityStep();
	void ValidateHandles();
	void EquilibrationStep();
	void StrainzeroingStep();
	void RemeshingStep(bool initializing = false);
	void UpdateObstacles(bool update_positions = true);
	void PhysicsStep(const std::vector<Constraint *> &cons);
	void DeleteConstraints(const std::vector<Constraint *> &cons);
	void StrainlimitingStep(const std::vector<Constraint *> &cons);
	std::vector<Constraint *> GetConstraints(bool include_proximity);

	void InitImGUI();
	void DumpClothMesh() const;
};

// Helper functions

template <typename Prim>
int size(const std::vector<Mesh *> &meshes);
template <typename Prim>
int get_index(const Prim *p, const std::vector<Mesh *> &meshes);
template <typename Prim>
Prim *get(int i, const std::vector<Mesh *> &meshes);
std::vector<Vec3> node_positions(const std::vector<Mesh *> &meshes);