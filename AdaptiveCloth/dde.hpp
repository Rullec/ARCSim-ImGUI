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

#ifndef DDE_HPP
#define DDE_HPP

#include "sparse.hpp"
#include "util.hpp"

typedef Vec<4> Vec4;
typedef Vec<3, float> Vec3f;
typedef Vec<6, float> Vec6f;

struct StretchingData
{
  Vec4 d[2][5];
};

struct StretchingSamples
{
  Vec4 s[40][40][40];
};

struct BendingData
{
  double d[3][5];
};

Vec4 stretching_stiffness(const Mat2x2 &G, const StretchingSamples &samples);

void evaluate_stretching_samples(StretchingSamples &samples, const StretchingData &data);

double bending_stiffness(const Edge *edge, int side, const BendingData &data, double initial_angle = 0);

enum eBendingMode
{
  DDE_BENDING_MODE = 0,
  LINEAR_ISOMETRIC_BENDING_MODE,
  LINEAR_ANISO_BENDING_MODE,
  NONLINEAR_BENDING_MODE,
  NUM_OF_BENDING_MODE
};
eBendingMode BuildBendingModeFromStr(std::string str);
std::string BuildBendingModeStr(eBendingMode mode);
Vec3f GetLinearBendingModulus();
void SetLinearBendingModulus(const Vec3f &val);
Vec6f GetNonlinearBendingModulus();
void SetNonlinearBendingModulus(const Vec6f &val);
#endif