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

#include "cloth.hpp"

void Cloth::ComputeMasses()
{
  for (int v = 0; v < mesh.verts.size(); v++)
    mesh.verts[v]->m = 0.0f;

  for (int n = 0; n < mesh.nodes.size(); n++)
    mesh.nodes[n]->m = 0.0f;

  for (int f = 0; f < mesh.faces.size(); f++)
  {
    Face *pFace = mesh.faces[f];

    pFace->m = pFace->a * materials[pFace->label]->density;

    for (int v = 0; v < 3; v++)
    {
      pFace->v[v]->m += pFace->m / 3.0f;

      pFace->v[v]->node->m += pFace->m / 3.0f;
    }
  }
}

void Cloth::SetDensity(int material_idx, float newDensity)
{
  float oldVal = materials[material_idx]->density;
  if (std::fabs(oldVal - newDensity) < 1e-6)
    return;
  else
  {
    materials[material_idx]->density = newDensity;
    this->ComputeMasses();
  }
}