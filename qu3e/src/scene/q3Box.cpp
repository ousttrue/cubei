//--------------------------------------------------------------------------------------------------
/**
@file	q3Box.cpp

@author	Randy Gaul
@date	10/10/2014

        Copyright (c) 2014 Randy Gaul http://www.randygaul.net

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:
          1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated but
is not required.
          2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
          3. This notice may not be removed or altered from any source
distribution.
*/
//--------------------------------------------------------------------------------------------------

#include "q3Box.h"
#include "../math/q3Vec3.h"
#include <q3Render.h>

q3Box::q3Box(const q3BoxDef &def) : def_(def) {}

bool q3Box::TestPoint(const q3Transform &tx, const q3Vec3 &p) const {
  q3Transform world = q3Mul(tx, def_.m_tx);
  q3Vec3 p0 = q3MulT(world, p);

  for (int i = 0; i < 3; ++i) {
    float d = p0[i];
    float ei = def_.m_e[i];

    if (d > ei || d < -ei) {
      return false;
    }
  }

  return true;
}

//--------------------------------------------------------------------------------------------------
bool q3Box::Raycast(const q3Transform &tx, q3RaycastData *raycast) const {
  q3Transform world = q3Mul(tx, def_.m_tx);
  q3Vec3 d = q3MulT(world.rotation, raycast->dir);
  q3Vec3 p = q3MulT(world, raycast->start);
  const float epsilon = float(1.0e-8);
  float tmin = 0;
  float tmax = raycast->t;

  // t = (e[ i ] - p.[ i ]) / d[ i ]
  float t0;
  float t1;
  q3Vec3 n0;

  for (int i = 0; i < 3; ++i) {
    // Check for ray parallel to and outside of AABB
    if (q3Abs(d[i]) < epsilon) {
      // Detect separating axes
      if (p[i] < -def_.m_e[i] || p[i] > def_.m_e[i]) {
        return false;
      }
    }

    else {
      float d0 = float(1.0) / d[i];
      float s = q3Sign(d[i]);
      float ei = def_.m_e[i] * s;
      q3Vec3 n = {};
      n[i] = -s;

      t0 = -(ei + p[i]) * d0;
      t1 = (ei - p[i]) * d0;

      if (t0 > tmin) {
        n0 = n;
        tmin = t0;
      }

      tmax = q3Min(tmax, t1);

      if (tmin > tmax) {
        return false;
      }
    }
  }

  raycast->normal = q3Mul(world.rotation, n0);
  raycast->toi = tmin;

  return true;
}

//--------------------------------------------------------------------------------------------------
void q3Box::ComputeAABB(const q3Transform &tx, q3AABB *aabb) const {
  q3Transform world = q3Mul(tx, def_.m_tx);

  q3Vec3 v[8] = {q3Vec3{-def_.m_e.x, -def_.m_e.y, -def_.m_e.z},
                 q3Vec3{-def_.m_e.x, -def_.m_e.y, def_.m_e.z},
                 q3Vec3{-def_.m_e.x, def_.m_e.y, -def_.m_e.z},
                 q3Vec3{-def_.m_e.x, def_.m_e.y, def_.m_e.z},
                 q3Vec3{def_.m_e.x, -def_.m_e.y, -def_.m_e.z},
                 q3Vec3{def_.m_e.x, -def_.m_e.y, def_.m_e.z},
                 q3Vec3{def_.m_e.x, def_.m_e.y, -def_.m_e.z},
                 q3Vec3{def_.m_e.x, def_.m_e.y, def_.m_e.z}};

  for (int i = 0; i < 8; ++i)
    v[i] = q3Mul(world, v[i]);

  q3Vec3 min{Q3_R32_MAX, Q3_R32_MAX, Q3_R32_MAX};
  q3Vec3 max{-Q3_R32_MAX, -Q3_R32_MAX, -Q3_R32_MAX};

  for (int i = 0; i < 8; ++i) {
    min = q3Min(min, v[i]);
    max = q3Max(max, v[i]);
  }

  aabb->min = min;
  aabb->max = max;
}

std::optional<q3MassData> q3Box::ComputeMass() const {
  if (def_.m_density == float(0.0)) {
    return {};
  }

  // Calculate inertia tensor
  float ex2 = float(4.0) * def_.m_e.x * def_.m_e.x;
  float ey2 = float(4.0) * def_.m_e.y * def_.m_e.y;
  float ez2 = float(4.0) * def_.m_e.z * def_.m_e.z;
  float mass =
      float(8.0) * def_.m_e.x * def_.m_e.y * def_.m_e.z * def_.m_density;
  float x = float(1.0 / 12.0) * mass * (ey2 + ez2);
  float y = float(1.0 / 12.0) * mass * (ex2 + ez2);
  float z = float(1.0 / 12.0) * mass * (ex2 + ey2);
  q3Mat3 I = q3Diagonal(x, y, z);

  // Transform tensor to local space
  I = def_.m_tx.rotation * I * q3Transpose(def_.m_tx.rotation);
  q3Mat3 identity = {};
  I += (identity * q3Dot(def_.m_tx.position, def_.m_tx.position) -
        q3OuterProduct(def_.m_tx.position, def_.m_tx.position)) *
       mass;

  return q3MassData{
      .inertia = I,
      .center = def_.m_tx.position,
      .mass = mass,
  };
}

//--------------------------------------------------------------------------------------------------
const int kBoxIndices[36] = {
    1 - 1, 7 - 1, 5 - 1, 1 - 1, 3 - 1, 7 - 1, 1 - 1, 4 - 1, 3 - 1,
    1 - 1, 2 - 1, 4 - 1, 3 - 1, 8 - 1, 7 - 1, 3 - 1, 4 - 1, 8 - 1,
    5 - 1, 7 - 1, 8 - 1, 5 - 1, 8 - 1, 6 - 1, 1 - 1, 5 - 1, 6 - 1,
    1 - 1, 6 - 1, 2 - 1, 2 - 1, 6 - 1, 8 - 1, 2 - 1, 8 - 1, 4 - 1};

//--------------------------------------------------------------------------------------------------
void q3Box::Render(const q3Transform &tx, bool awake, q3Render *render) const {
  q3Transform world = q3Mul(tx, def_.m_tx);

  q3Vec3 vertices[8] = {{-def_.m_e.x, -def_.m_e.y, -def_.m_e.z},
                        {-def_.m_e.x, -def_.m_e.y, def_.m_e.z},
                        {-def_.m_e.x, def_.m_e.y, -def_.m_e.z},
                        {-def_.m_e.x, def_.m_e.y, def_.m_e.z},
                        {def_.m_e.x, -def_.m_e.y, -def_.m_e.z},
                        {def_.m_e.x, -def_.m_e.y, def_.m_e.z},
                        {def_.m_e.x, def_.m_e.y, -def_.m_e.z},
                        {def_.m_e.x, def_.m_e.y, def_.m_e.z}};

  for (int i = 0; i < 36; i += 3) {
    q3Vec3 a = q3Mul(world, vertices[kBoxIndices[i]]);
    q3Vec3 b = q3Mul(world, vertices[kBoxIndices[i + 1]]);
    q3Vec3 c = q3Mul(world, vertices[kBoxIndices[i + 2]]);

    q3Vec3 n = q3Normalize(q3Cross(b - a, c - a));

    // render->SetPenColor( 0.2f, 0.4f, 0.7f, 0.5f );
    // render->SetPenPosition( a.x, a.y, a.z );
    // render->Line( b.x, b.y, b.z );
    // render->Line( c.x, c.y, c.z );
    // render->Line( a.x, a.y, a.z );

    render->SetTriNormal(n.x, n.y, n.z);
    render->Triangle(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
  }
}

void q3Box::Dump(FILE *file, int index) const {
  fprintf(file, "\t{\n");
  fprintf(file, "\t\tq3BoxDef sd;\n");
  def_.Dump(file);
  fprintf(file, "\t\tbodies[ %d ]->AddBox( sd );\n", index);
  fprintf(file, "\t}\n");
}

void q3BoxDef::Dump(FILE *file) const {
  fprintf(file, "\t\tsd.SetFriction( float( %.15lf ) );\n", m_friction);
  fprintf(file, "\t\tsd.SetRestitution( float( %.15lf ) );\n", m_restitution);
  fprintf(file, "\t\tsd.SetDensity( float( %.15lf ) );\n", m_density);
  fprintf(file, "\t\tsd.SetSensor( bool( %d ) );\n", m_sensor);
  fprintf(file, "\t\tq3Transform boxTx;\n");
  q3Transform boxTx = m_tx;
  q3Vec3 xAxis = boxTx.rotation.ex;
  q3Vec3 yAxis = boxTx.rotation.ey;
  q3Vec3 zAxis = boxTx.rotation.ez;
  fprintf(file,
          "\t\tq3Vec3 xAxis( float( %.15lf ), float( %.15lf ), float( %.15lf "
          ") );\n",
          xAxis.x, xAxis.y, xAxis.z);
  fprintf(file,
          "\t\tq3Vec3 yAxis( float( %.15lf ), float( %.15lf ), float( %.15lf "
          ") );\n",
          yAxis.x, yAxis.y, yAxis.z);
  fprintf(file,
          "\t\tq3Vec3 zAxis( float( %.15lf ), float( %.15lf ), float( %.15lf "
          ") );\n",
          zAxis.x, zAxis.y, zAxis.z);
  fprintf(file, "\t\tboxTx.rotation.SetRows( xAxis, yAxis, zAxis );\n");
  fprintf(file,
          "\t\tboxTx.position.Set( float( %.15lf ), float( %.15lf ), float( "
          "%.15lf ) );\n",
          boxTx.position.x, boxTx.position.y, boxTx.position.z);
  fprintf(file,
          "\t\tsd.Set( boxTx, q3Vec3( float( %.15lf ), float( %.15lf ), "
          "float( %.15lf ) ) );\n",
          m_e.x * 2.0f, m_e.y * 2.0f, m_e.z * 2.0f);
}
