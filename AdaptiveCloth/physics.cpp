/*
  Copyright ©2013 The Regents of the University of California
  (Regents). All Rights Reserved. Permission to use, copy, modify, and
  distribute this software and its documentation for educational,
  research, and not-for-profit purposes, without fee and without a
  signed licensing agreement, is hereby granted, provided that the
  above copyright notice, this paragraph and the following two
  paragraphs appear in all copies, modifications, and
  distributions. Contact The Office of Technology Licensing, UC
  Berkeley, 2150 Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620,
  (510) 643-7201, for commercial licensing opportunities.

  IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT,
  INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
  LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
  DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY
  OF SUCH DAMAGE.

  REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING
  DOCUMENTATION, IF ANY, PROVIDED HEREUNDER IS PROVIDED "AS
  IS". REGENTS HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
*/

#include "physics.hpp"

#include "blockvectors.hpp"
#include "collisionutil.hpp"
#include "sparse.hpp"
#include "taucs.hpp"

using namespace std;

static const bool verbose = false;

static const vector<Cloth::Material *> *materials;

typedef Mat<9, 9> Mat9x9;
typedef Mat<9, 6> Mat9x6;
typedef Mat<6, 6> Mat6x6;
typedef Mat<4, 6> Mat4x6;
typedef Mat<3, 4> Mat3x4;
typedef Mat<4, 9> Mat4x9;
typedef Vec<9> Vec9;

// A kronecker B = [a11 B, a12 B, ..., a1n B;
//                  a21 B, a22 B, ..., a2n B;
//                   ... ,  ... , ...,  ... ;
//                  am1 B, am2 B, ..., amn B]
template <int m, int n, int p, int q>
Mat<m * p, n * q> kronecker(const Mat<m, n> &A, const Mat<p, q> &B)
{
	Mat<m * p, n * q> C;
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			for (int k = 0; k < p; k++)
				for (int l = 0; l < q; l++)
					C(i * p + k, j * q + l) = A(i, j) * B(k, l);
	return C;
}

template <int m>
Mat<m, 1> colmat(const Vec<m> &v)
{
	Mat<1, m> A;
	for (int i = 0; i < m; i++)
		A(i, 0) = v[i];
	return A;
}
template <int n>
Mat<1, n> rowmat(const Vec<n> &v)
{
	Mat<1, n> A;
	for (int i = 0; i < n; i++)
		A(0, i) = v[i];
	return A;
}

template <Space s>
double stretching_energy(const Face *face)
{
	Mat3x2 F = derivative(pos<s>(face->v[0]->node), pos<s>(face->v[1]->node),
						  pos<s>(face->v[2]->node), face);
	Mat2x2 G = (F.t() * F - Mat2x2(1)) / 2.;
	Vec4 k = stretching_stiffness(G, (*::materials)[face->label]->stretching);
	double weakening = (*::materials)[face->label]->weakening;
	k *= 1 / (1 + weakening * face->damage);
	return face->a * (k[0] * sq(G(0, 0)) + k[2] * sq(G(1, 1)) + 2 * k[1] * G(0, 0) * G(1, 1) + k[3] * sq(G(0, 1))) / 2.;
}

template <Space s>
pair<Mat9x9, Vec9> stretching_force(const Face *face)
{
	Mat3x2 F = derivative(pos<s>(face->v[0]->node), pos<s>(face->v[1]->node),
						  pos<s>(face->v[2]->node), face);
	Mat2x2 G = (F.t() * F - Mat2x2(1)) / 2.;
	Vec4 k = stretching_stiffness(G, (*::materials)[face->label]->stretching);
	double weakening = (*::materials)[face->label]->weakening;
	k *= 1 / (1 + weakening * face->damage);
	// eps = 1/2(F'F - I) = 1/2([x_u^2 & x_u x_v \\ x_u x_v & x_v^2] - I)
	// e = 1/2 k0 eps00^2 + k1 eps00 eps11 + 1/2 k2 eps11^2 + k3 eps01^2
	// grad e = k0 eps00 grad eps00 + ...
	//        = k0 eps00 Du' x_u + ...
	Mat2x3 D = derivative(face);
	Vec3 du = D.row(0), dv = D.row(1);
	Mat<3, 9> Du = kronecker(rowmat(du), Mat3x3(1)),
			  Dv = kronecker(rowmat(dv), Mat3x3(1));
	const Vec3 &xu = F.col(0), &xv = F.col(1); // should equal Du*mat_to_vec(X)
	Vec9 fuu = Du.t() * xu, fvv = Dv.t() * xv, fuv = (Du.t() * xv + Dv.t() * xu) / 2.;
	Vec9 grad_e = k[0] * G(0, 0) * fuu + k[2] * G(1, 1) * fvv + k[1] * (G(0, 0) * fvv + G(1, 1) * fuu) + 2 * k[3] * G(0, 1) * fuv;
	Mat9x9 hess_e = k[0] * (outer(fuu, fuu) + max(G(0, 0), 0.) * Du.t() * Du) + k[2] * (outer(fvv, fvv) + max(G(1, 1), 0.) * Dv.t() * Dv) + k[1] * (outer(fuu, fvv) + max(G(0, 0), 0.) * Dv.t() * Dv + outer(fvv, fuu) + max(G(1, 1), 0.) * Du.t() * Du) + 2. * k[3] * (outer(fuv, fuv));
	// ignoring G(0,1)*(Du.t()*Dv+Dv.t()*Du)/2. term
	// because may not be positive definite
	return make_pair(-face->a * hess_e, -face->a * grad_e);
}

typedef Mat<12, 12> Mat12x12;
typedef Vec<12> Vec12;

template <Space s>
double bending_energy(const Edge *edge)
{
	const Face *face0 = edge->adjf[0];
	const Face *face1 = edge->adjf[1];

	if (!face0 || !face1)
		return 0;

	double theta = dihedral_angle<s>(edge);

	double a = face0->a + face1->a;

	const BendingData &bend0 = (*::materials)[face0->label]->bending;
	const BendingData &bend1 = (*::materials)[face1->label]->bending;

	double ke = min(bending_stiffness(edge, 0, bend0),
					bending_stiffness(edge, 1, bend1));

	double weakening = max((*::materials)[face0->label]->weakening, (*::materials)[face1->label]->weakening);

	ke *= 1 / (1 + weakening * edge->damage);

	double shape = sq(edge->l) / (2 * a);

	return ke * shape * sq(theta - edge->theta_ideal) / 4;
}

double distance(const Vec3 &x, const Vec3 &a, const Vec3 &b)
{
	Vec3 e = b - a;
	Vec3 xp = e * dot(e, x - a) / dot(e, e);
	// return norm((x-a)-xp);
	return max(norm((x - a) - xp), 1e-3 * norm(e));
}

Vec2 barycentric_weights(const Vec3 &x, const Vec3 &a, const Vec3 &b)
{
	Vec3 e = b - a;
	double t = dot(e, x - a) / dot(e, e);
	return Vec2(1 - t, t);
}
double cot_vec(const Vec3 &v0, const Vec3 &v1)
{
	double deno = norm(cross(v0, v1));
	if (std::fabs(deno) < 1e-10)
	{
		deno = (deno > 0 ? 1 : -1) * 1e-10;
	}
	return dot(v0, v1) / deno;
}
typedef Mat<3, 12> Mat3x12;
std::vector<Mat12x12> gQBendingHessianArray = {};
static std::vector<bool> gQBendingCalced = {};
template <Space s>
pair<Mat12x12, Vec12> bending_force_qbending(const Edge *edge)
{
	// 1. calculate the bs of edge
	const Face *face0 = edge->adjf[0], *face1 = edge->adjf[1];
	const BendingData &bend0 = (*::materials)[face0->label]->bending,
					  &bend1 = (*::materials)[face1->label]->bending;
	double bs = std::min(bending_stiffness(edge, 0, bend0),
						 bending_stiffness(edge, 1, bend1));
	Mat12x12 hessian;

	// 2.1 get v0, v1, v2, v3
	auto v0 = edge->n[0];
	auto v1 = edge->n[1];
	auto v2 = edge_opp_vert(edge, 0)->node;
	auto v3 = edge_opp_vert(edge, 1)->node;

	Vec3 x0 = pos<s>(v0),
		 x1 = pos<s>(v1),
		 x2 = pos<s>(v2),
		 x3 = pos<s>(v3);
	Vec12 x_vec = mat_to_vec(Mat3x4(x0, x1, x2, x3));
	// not avaliable, begin to calc
	if (gQBendingHessianArray.size() <= edge->index || gQBendingCalced[edge->index] == false)
	{
		gQBendingHessianArray.resize(edge->index + 1);
		gQBendingCalced.resize(edge->index + 1);
		gQBendingCalced[edge->index] = true;
		// 2.2 get e0, e1, e2, e3
		/*
		e0 = x1 - x0
		e1 = x2 - x0
		e2 = x3 - x0
		e3 = x2 - x1
		e4 = x3 - x1
		*/
		Vec3 e0 = x1 - x0;
		Vec3 e1 = x2 - x0;
		Vec3 e2 = x3 - x0;
		Vec3 e3 = x2 - x1;
		Vec3 e4 = x3 - x1;

		// 2.3 calculate coef;
		double c01 = cot_vec(e0, e1);
		double c02 = cot_vec(e0, e2);
		double c03 = cot_vec(-e0, e3);
		double c04 = cot_vec(-e0, e4);
		/*
			a = c_{03} + c_{04}
			b = c_{01} + c_{02}
			c = -c_{01} - c_{03}
			d = -c_{02} - c_{04}
		*/
		double coef[4] = {c03 + c04,
						  c01 + c02,
						  -c01 - c03,
						  -c02 - c04};

		// 2.4 get vector K =[aI3, bI3, cI3, dI3]
		Mat3x12 K;
		// a part
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 3; j++)
				K(j, 3 * i + j) = coef[i];
		}
		double a = face0->a + face1->a; // area
		double shape_factor = 6.0 / a;
		// shape_factor /= 3;
		// 3. calculate hessian
		// 4. calculate force
		hessian = shape_factor * transpose(K) * K;
		gQBendingHessianArray[edge->index] = hessian;
	}

	hessian = bs * gQBendingHessianArray[edge->index];

	Vec12 force = hessian * x_vec;
	return make_pair(-hessian, -force);
}

template <Space s>
pair<Mat12x12, Vec12> bending_force_dihedral(const Edge *edge)
{
	const Face *face0 = edge->adjf[0], *face1 = edge->adjf[1];
	if (!face0 || !face1)
		return make_pair(Mat12x12(0), Vec12(0));
	double theta = dihedral_angle<s>(edge); // \theta \in [-\pi, \pi];
	double a = face0->a + face1->a;			// area
	Vec3 x0 = pos<s>(edge->n[0]),
		 x1 = pos<s>(edge->n[1]),
		 x2 = pos<s>(edge_opp_vert(edge, 0)->node),
		 x3 = pos<s>(edge_opp_vert(edge, 1)->node);
	double h0 = distance(x2, x0, x1), h1 = distance(x3, x0, x1); // h0, h1
	Vec3 n0 = nor<s>(face0), n1 = nor<s>(face1);
	Vec2 w_f0 = barycentric_weights(x2, x0, x1),
		 w_f1 = barycentric_weights(x3, x0, x1);
	Vec12 dtheta = mat_to_vec(Mat3x4(-(w_f0[0] * n0 / h0 + w_f1[0] * n1 / h1),
									 -(w_f0[1] * n0 / h0 + w_f1[1] * n1 / h1),
									 n0 / h0,
									 n1 / h1));
	const BendingData &bend0 = (*::materials)[face0->label]->bending,
					  &bend1 = (*::materials)[face1->label]->bending;
	double ke = min(bending_stiffness(edge, 0, bend0),
					bending_stiffness(edge, 1, bend1));
	double weakening = max((*::materials)[face0->label]->weakening,
						   (*::materials)[face1->label]->weakening);
	ke *= 1 / (1 + weakening * edge->damage);
	double shape = sq(edge->l) / (2 * a);

	// if (edge->index % 100 == 0)
	// {
	// 	printf("cur ke %.1e, now we apply it 4*6 to %.1e\n", ke, ke * 4 * 6);
	// }
	ke *= 4 * 6;

	// hessian is dtheta * dtheta.T / 2
	// force is theta * dtheta / 2
	return make_pair(-ke * shape * outer(dtheta, dtheta) / 2.,
					 -ke * shape * (theta - edge->theta_ideal) * dtheta / 2.);
}
bool gUseQBending = false;

template <Space s>
pair<Mat12x12, Vec12> bending_force(const Edge *edge)
{
	if (gUseQBending == true)
	{
		return bending_force_qbending<s>(edge);
	}
	else
	{
		return bending_force_dihedral<s>(edge);
	}
}

template <int m, int n>
Mat<3, 3> submat3(const Mat<m, n> &A, int i, int j)
{
	Mat3x3 Asub;
	for (int k = 0; k < 3; k++)
		for (int l = 0; l < 3; l++)
			Asub(k, l) = A(i * 3 + k, j * 3 + l);
	return Asub;
}

template <int n>
Vec<3> subvec3(const Vec<n> &b, int i)
{
	Vec3 bsub;
	for (int k = 0; k < 3; k++)
		bsub[k] = b[i * 3 + k];
	return bsub;
}

template <int m>
void add_submat(const Mat<m * 3, m * 3> &Asub, const Vec<m, int> &ix, SpMat<Mat3x3> &A)
{
	for (int i = 0; i < m; i++)
		for (int j = 0; j < m; j++)
			A(ix[i], ix[j]) += submat3(Asub, i, j);
}

template <int m>
void add_subvec(const Vec<m * 3> &bsub, const Vec<m, int> &ix, vector<Vec3> &b)
{
	for (int i = 0; i < m; i++)
		b[ix[i]] += subvec3(bsub, i);
}

Vec<3, int> indices(const Node *n0, const Node *n1, const Node *n2)
{
	Vec<3, int> ix;
	ix[0] = n0->index;
	ix[1] = n1->index;
	ix[2] = n2->index;
	return ix;
}

Vec<4, int> indices(const Node *n0, const Node *n1,
					const Node *n2, const Node *n3)
{
	Vec<4, int> ix;
	ix[0] = n0->index;
	ix[1] = n1->index;
	ix[2] = n2->index;
	ix[3] = n3->index;
	return ix;
}

// aa: use 3-compressed sparse matrix
#define USE_SPARSE3

template <Space s>
double internal_energy(const Cloth &cloth)
{
	const Mesh &mesh = cloth.mesh;
	::materials = &cloth.materials;

	double E = 0;

	for (int f = 0; f < mesh.faces.size(); f++)
	{
		E += stretching_energy<s>(mesh.faces[f]);
	}

	for (int e = 0; e < mesh.edges.size(); e++)
	{
		E += bending_energy<s>(mesh.edges[e]);
	}
	return E;
}

template double internal_energy<PS>(const Cloth &);
template double internal_energy<WS>(const Cloth &);

// A = dt^2 J + dt damp J
// b = dt f + dt^2 J v + dt damp J v

template <Space s>
void add_internal_forces(const Cloth &cloth, SpMat<Mat3x3> &A,
						 vector<Vec3> &b, double dt)
{
	const Mesh &mesh = cloth.mesh;
	::materials = &cloth.materials;
	for (int f = 0; f < mesh.faces.size(); f++)
	{
		const Face *face = mesh.faces[f];
		const Node *n0 = face->v[0]->node, *n1 = face->v[1]->node,
				   *n2 = face->v[2]->node;
		Vec9 vs = mat_to_vec(Mat3x3(n0->v, n1->v, n2->v));
		pair<Mat9x9, Vec9> membF = stretching_force<s>(face);
		Mat9x9 J = membF.first;
		Vec9 F = membF.second;
		if (dt == 0)
		{
			add_submat(-J, indices(n0, n1, n2), A);
			add_subvec(F, indices(n0, n1, n2), b);
		}
		else
		{
			double damping = (*::materials)[face->label]->damping;
			// printf("[fint] stretch f %d damping %.3f\n", f, damping);
			add_submat(-dt * (dt + damping) * J, indices(n0, n1, n2), A);
			add_subvec(dt * (F + (dt + damping) * J * vs), indices(n0, n1, n2), b);
		}
	}
	for (int e = 0; e < mesh.edges.size(); e++)
	{
		const Edge *edge = mesh.edges[e];
		if (!edge->adjf[0] || !edge->adjf[1])
			continue;
		pair<Mat12x12, Vec12> bendF = bending_force<s>(edge);
		const Node *n0 = edge->n[0],
				   *n1 = edge->n[1],
				   *n2 = edge_opp_vert(edge, 0)->node,
				   *n3 = edge_opp_vert(edge, 1)->node;
		Vec12 vs = mat_to_vec(Mat3x4(n0->v, n1->v, n2->v, n3->v));
		Mat12x12 J = bendF.first;
		Vec12 F = bendF.second;
		if (dt == 0)
		{
			add_submat(-J, indices(n0, n1, n2, n3), A);
			add_subvec(F, indices(n0, n1, n2, n3), b);
		}
		else
		{
			double damping = ((*::materials)[edge->adjf[0]->label]->damping +
							  (*::materials)[edge->adjf[1]->label]->damping) /
							 2.;
			// printf("[fint] bending e %d damping %.3f\n", e, damping);
			add_submat(-dt * (dt + damping) * J, indices(n0, n1, n2, n3), A);
			add_subvec(dt * (F + (dt + damping) * J * vs), indices(n0, n1, n2, n3), b);
		}
	}
}
template void add_internal_forces<PS>(const Cloth &, SpMat<Mat3x3> &,
									  vector<Vec3> &, double);
template void add_internal_forces<WS>(const Cloth &, SpMat<Mat3x3> &,
									  vector<Vec3> &, double);

bool contains(const Mesh &mesh, const Node *node)
{
	return node->index < mesh.nodes.size() && mesh.nodes[node->index] == node;
}

double constraint_energy(const vector<Constraint *> &cons)
{
	double E = 0;
	for (int c = 0; c < cons.size(); c++)
	{
		double value = cons[c]->value();
		double e = cons[c]->energy(value);
		E += e;
	}
	return E;
}

void add_constraint_forces(const Cloth &cloth, const vector<Constraint *> &cons,
						   SpMat<Mat3x3> &A, vector<Vec3> &b, double dt)
{
	const Mesh &mesh = cloth.mesh;
	for (int c = 0; c < cons.size(); c++)
	{
		double value = cons[c]->value();
		double g = cons[c]->energy_grad(value);
		double h = cons[c]->energy_hess(value);
		MeshGrad grad = cons[c]->gradient();
		// f = -g*grad
		// J = -h*outer(grad,grad)
		double v_dot_grad = 0;
		for (MeshGrad::iterator it = grad.begin(); it != grad.end(); it++)
		{
			const Node *node = it->first;
			v_dot_grad += dot(it->second, node->v);
		}
		for (MeshGrad::iterator it = grad.begin(); it != grad.end(); it++)
		{
			const Node *nodei = it->first;
			if (!contains(mesh, nodei))
				continue;
			int ni = nodei->index;
			for (MeshGrad::iterator jt = grad.begin(); jt != grad.end(); jt++)
			{
				const Node *nodej = jt->first;
				if (!contains(mesh, nodej))
					continue;
				int nj = nodej->index;
				if (dt == 0)
					A(ni, nj) += h * outer(it->second, jt->second);
				else
					A(ni, nj) += dt * dt * h * outer(it->second, jt->second);
			}
			if (dt == 0)
				b[ni] -= g * it->second;
			else
				b[ni] -= dt * (g + dt * h * v_dot_grad) * it->second;
		}
	}
}

void add_friction_forces(const Cloth &cloth, const vector<Constraint *> cons,
						 SpMat<Mat3x3> &A, vector<Vec3> &b, double dt)
{
	const Mesh &mesh = cloth.mesh;
	for (int c = 0; c < cons.size(); c++)
	{
		MeshHess jac;
		MeshGrad force = cons[c]->friction(dt, jac);
		for (MeshGrad::iterator it = force.begin(); it != force.end(); it++)
		{
			const Node *node = it->first;
			if (!contains(mesh, node))
				continue;
			b[node->index] += dt * it->second;
		}
		for (MeshHess::iterator it = jac.begin(); it != jac.end(); it++)
		{
			const Node *nodei = it->first.first, *nodej = it->first.second;
			if (!contains(mesh, nodei) || !contains(mesh, nodej))
				continue;
			A(nodei->index, nodej->index) -= dt * it->second;
		}
	}
}

void project_outside(Mesh &mesh, const vector<Constraint *> &cons);
#include "utils/TimeUtil.hpp"
void implicit_update(Cloth &cloth, const vector<Vec3> &fext,
					 const vector<Mat3x3> &Jext,
					 const vector<Constraint *> &cons, double dt,
					 bool update_positions)
{
	Mesh &mesh = cloth.mesh;
	vector<Vert *>::iterator vert_it;
	vector<Face *>::iterator face_it;
	int nn = mesh.nodes.size();
	// M Dv/Dt = F (x + Dx) = F (x + Dt (v + Dv))
	// Dv = Dt (M - Dt2 F)i F (x + Dt v)
	// A = M - Dt2 F
	// b = Dt F (x + Dt v)
	SpMat<Mat3x3> A(nn, nn);
	vector<Vec3> b(nn, Vec3(0));
	for (int n = 0; n < mesh.nodes.size(); n++)
	{
		const Node *node = mesh.nodes[n];
		A(n, n) += Mat3x3(node->m) - dt * dt * Jext[n];
		b[n] += dt * fext[n];
	}
	cTimeUtil::Begin("fint");
	add_internal_forces<WS>(cloth, A, b, dt);
	add_constraint_forces(cloth, cons, A, b, dt);
	add_friction_forces(cloth, cons, A, b, dt);
	cTimeUtil::End("fint");
	cTimeUtil::Begin("solve");
	vector<Vec3> dv = taucs_linear_solve(A, b);
	cTimeUtil::End("solve");

	cTimeUtil::Begin("post_solve");
	for (int n = 0; n < mesh.nodes.size(); n++)
	{
		Node *node = mesh.nodes[n];
		node->v += dv[n];
		if (update_positions)
			node->x += node->v * dt;
		node->acceleration = dv[n] / dt;
	}
	project_outside(cloth.mesh, cons);
	compute_ws_data(mesh);
	cTimeUtil::End("post_solve");
}

Vec3 wind_force(const Face *face, const Wind &wind)
{
	Vec3 vface = (face->v[0]->node->v + face->v[1]->node->v + face->v[2]->node->v) / 3.;
	Vec3 vrel = wind.velocity - vface;
	double vn = dot(face->n, vrel);
	Vec3 vt = vrel - vn * face->n;
	return wind.density * face->a * abs(vn) * vn * face->n + wind.drag * face->a * vt;
}

void add_external_forces(const Cloth &cloth, const Vec3 &gravity,
						 const Wind &wind, vector<Vec3> &fext,
						 vector<Mat3x3> &Jext)
{
	const Mesh &mesh = cloth.mesh;
	for (int n = 0; n < mesh.nodes.size(); n++)
		fext[n] += mesh.nodes[n]->m * gravity;
	for (int f = 0; f < mesh.faces.size(); f++)
	{
		const Face *face = mesh.faces[f];
		Vec3 fw = wind_force(face, wind);
		for (int v = 0; v < 3; v++)
			fext[face->v[v]->node->index] += fw / 3.;
	}
}

void add_morph_forces(const Cloth &cloth, const Morph &morph, double t,
					  double dt, vector<Vec3> &fext, vector<Mat3x3> &Jext)
{
	const Mesh &mesh = cloth.mesh;
	for (int v = 0; v < mesh.verts.size(); v++)
	{
		const Vert *vert = mesh.verts[v];
		Vec3 x = morph.pos(t, vert->u);
		double stiffness = exp(morph.log_stiffness.pos(t));
		Vec3 n = vert->node->n;
		double s = stiffness * vert->a;
		// // lower stiffness in tangential direction
		// Mat3x3 k = s*outer(n,n) + (s/10)*(Mat3x3(1) - outer(n,n));
		Mat3x3 k = Mat3x3(s);
		double c = sqrt(s * vert->m); // subcritical damping
		Mat3x3 d = c / s * k;
		fext[vert->node->index] -= k * (vert->node->x - x);
		fext[vert->node->index] -= d * vert->node->v;
		Jext[vert->node->index] -= k + d / dt;
	}
}

void project_outside(Mesh &mesh, const vector<Constraint *> &cons)
{
	int nn = mesh.nodes.size();
	vector<double> w(nn, 0);
	vector<Vec3> dx(nn, Vec3(0));
	for (int c = 0; c < cons.size(); c++)
	{
		MeshGrad dxc = cons[c]->project();
		for (MeshGrad::iterator it = dxc.begin(); it != dxc.end(); it++)
		{
			const Node *node = it->first;
			double wn = norm2(it->second);
			int n = node->index;
			if (n >= mesh.nodes.size() || mesh.nodes[n] != node)
				continue;
			w[n] += wn;
			dx[n] += wn * it->second;
		}
	}
	for (int n = 0; n < nn; n++)
	{
		if (w[n] == 0)
			continue;
		mesh.nodes[n]->x += dx[n] / w[n];
	}
}
