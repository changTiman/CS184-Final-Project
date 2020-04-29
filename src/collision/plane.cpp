#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "../leak_fix.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with planes.
    
    double last_dir = dot(point - pm.last_position, normal);
    double curr_dir = dot(point - pm.position, normal);

    // for some reason SURFACE_OFFSET is too small relative to cloth thickness so even when point masses are SURFACE_OFFSET 
    // dist away from plane, they will clip through the plane. This if statements prevents clipping by making sure that 
    // the pm.position should never get closer than 10 * SURFACE_OFFSET.
    double curr_dist_to_plane = (curr_dir * normal).norm();
    if ((curr_dist_to_plane <= 10 * SURFACE_OFFSET) && 
        ((last_dir > 0 && curr_dir > 0) || (last_dir < 0 && curr_dir < 0))) {
        pm.position = pm.last_position;
    }

    else if ((last_dir > 0 && curr_dir <= 0) || (last_dir < 0 && curr_dir >= 0)) {
        Vector3D tangent = pm.position + curr_dir * normal;
        int dir = (last_dir < 0) ? 1 : -1;
        Vector3D offset_tangent = tangent + dir * SURFACE_OFFSET * normal;
        Vector3D correction = offset_tangent - pm.last_position;

        Vector3D potential_position = pm.last_position + correction * (1 - friction);

        pm.position = potential_position;


        // this doesn't do what i want
        //double pot_dist = (dot(potential_position - point, normal) * normal).norm();
        //if (pot_dist > 2 * SURFACE_OFFSET) {
        //    pm.position = potential_position;
        //}
        //else {
        //    // keeps moving down even after this update
        //    pm.position = pm.last_position;
        //    //pm.position = offset_tangent;
        //}

        // only update if correction vector points same direction as last direction
        // this only happends if the last position is below offset_tangent
        //if ((dot(correction, normal) < 0 && -dir < 0) || (dot(correction, normal) > 0 && -dir > 0)) {
        //    pm.position = pm.last_position + correction * (1 - friction);
        //}
        //else {
        //    //pm.position = pm.last_position;
        //    pm.position = offset_tangent;
        //}
    }
}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 2 * (sCross + sParallel);
  positions.col(1) << sPoint + 2 * (sCross - sParallel);
  positions.col(2) << sPoint + 2 * (-sCross + sParallel);
  positions.col(3) << sPoint + 2 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
#ifdef LEAK_PATCH_ON
  shader.freeAttrib("in_position");
  if (shader.attrib("in_normal", false) != -1) {
    shader.freeAttrib("in_normal");
  }
#endif
}
