#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
    double width_offset = width / num_width_points;
    double height_offset = height / num_height_points;

    for (int c = 0; c < num_height_points; c++) {
        for (int r = 0; r < num_width_points; r++) {
            double x, y, z;
            bool pin;
            
            x = r * width_offset;

            if (orientation == HORIZONTAL) {
                y = 1.0;
                z = c * height_offset;              
            }
            else {
                // vertical
                y = c * height_offset;
                z = (double)rand() / (RAND_MAX * 500) - (1.0 / 1000);
                //z = 0;
            }

            vector<int> v = { r, c };
            bool p = std::find(pinned.begin(), pinned.end(), v) != pinned.end();

            point_masses.emplace_back(PointMass(Vector3D(x, y, z), p));
        }
    }

    PointMass *ptr = &point_masses[0];

    // set spring constraints
    for (int c = 0; c < num_height_points; c++) {
        for (int r = 0; r < num_width_points; r++) {
            PointMass* curr = ptr + c * num_width_points + r;

            // Structural
            if (r != 0) {
                springs.emplace_back(Spring(curr, curr - 1, STRUCTURAL));
            }
            if (c != 0) {
                springs.emplace_back(Spring(curr, curr - num_width_points, STRUCTURAL));
            }

            // Shearing
            if (r != 0 && c != 0) {
                springs.emplace_back(Spring(curr, curr - num_width_points - 1, SHEARING));
            }
            if (r != num_width_points - 1 && c != 0) {
                springs.emplace_back(Spring(curr, curr - num_width_points + 1, SHEARING));
            }

            // Bending
            if (r > 1) {
                springs.emplace_back(Spring(curr, curr - 2, BENDING));
            }
            if (c > 1) {
                springs.emplace_back(Spring(curr, curr - 2 * num_width_points, BENDING));
            }
        }
    }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D tot_ext_force = Vector3D();
  for (auto f : external_accelerations) {
      tot_ext_force += f;
  }
  tot_ext_force *= mass;

  for (auto &pm : point_masses) {
      // clear previous forces by just setting to tot_ext_forces
      pm.forces = tot_ext_force;
  }

  for (auto &s : springs) {
      double sp_cor_f = 0.0;
      if (cp->enable_structural_constraints && s.spring_type == STRUCTURAL) {
          sp_cor_f = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length);
      }
      else if (cp->enable_shearing_constraints && s.spring_type == SHEARING) {
          sp_cor_f = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length);
      }
      else if (cp->enable_bending_constraints && s.spring_type == BENDING) {
          sp_cor_f = cp->ks * 0.2 * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length);
      }
      Vector3D dir_ab = s.pm_b->position - s.pm_a->position;
      s.pm_a->forces += sp_cor_f * dir_ab;
      s.pm_b->forces -= sp_cor_f * dir_ab;
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (auto &pm : point_masses) {
      if (!pm.pinned) {
          Vector3D last_pos = pm.position;
          Vector3D acc = pm.forces / mass;
          pm.position += (1 - cp->damping / 100) * (pm.position - pm.last_position) + acc * delta_t * delta_t;
          pm.last_position = last_pos;
      }
  }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (auto &pm : point_masses) {
      self_collide(pm, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
  for (auto &pm : point_masses) {
      for (auto &&cm : *collision_objects) {
          cm->collide(pm);
      }
  }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (auto &s : springs) {
      Vector3D ab = s.pm_b->position - s.pm_a->position;
      double max_len = 1.1 * s.rest_length;
      if (ab.norm() > max_len) {
          if (!s.pm_a->pinned || !s.pm_b->pinned) {
              if (s.pm_a->pinned) {
                  s.pm_b->position = s.pm_a->position + ab.unit() * max_len;
              }
              else if (s.pm_b->pinned) {
                  s.pm_a->position = s.pm_b->position - ab.unit() * max_len;
              }
              else {
                  // neither pinned
                  double half_correction = (ab.norm() - max_len) / 2;
                  s.pm_a->position += ab.unit() * half_correction;
                  s.pm_b->position -= ab.unit() * half_correction;
              }
          }
      }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (auto &pm : point_masses) {
      float hash = hash_position(pm.position);

      auto search = map.find(hash);
      if (search == map.end()) {
          vector<PointMass*> *new_vec = new vector<PointMass*>;
          new_vec->emplace_back(&pm);

          map.insert(std::make_pair(hash, new_vec));
      }
      else {
          search->second->emplace_back(&pm);
      }
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
    float hash = hash_position(pm.position);
    Vector3D correction = Vector3D();

    for (auto &&cp : *map.at(hash)) {
        Vector3D dir = pm.position - cp->position;

        if (dir.norm() > 0 && dir.norm() < 2 * thickness) {
            correction += (cp->position + dir.unit() * 2 * thickness) - pm.position;
        }
    }

    pm.position += correction / simulation_steps;
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    double w = 3 * width / num_width_points;
    double h = 3 * height / num_height_points;
    double t = max(w, h);

    Vector3D new_coord = Vector3D(floor(pos.x / w), floor(pos.y / h), floor(pos.z / t));

    int hash = ((int)new_coord.x * 73856093) ^ ((int)new_coord.y * 19349663) ^ ((int)new_coord.z * 83492791);

  return fmod(hash, 2000); 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
