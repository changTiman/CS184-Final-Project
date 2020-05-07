#ifndef FIRE_H
#define FIRE_H

#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"

using namespace CGL;
using namespace std;

struct FireVoxel {
    FireVoxel() {}
    FireVoxel(double phi, double temp, double rho, double pres, Vector3D pos) : phi(phi), temp(temp), rho(rho), pres(pres), position(pos), prev_phi(phi) {}
    ~FireVoxel() {}

    // phi will define whether rho and p are for fuel or hot gaseous products
    double phi;         // implicit surface definition:  
                        // positive in the region of space filled with fuel,
                        // negative elsewhere, and zero at the reaction zone
    double prev_phi;    // phi at last time step
    double temp;        // temperature
    double rho;         // density
    double pres;        // pressure

    bool fixed_phi = false;

    // Position
    Vector3D position;

    // Velocity components
    double *u_down;  // i - 1/2 velocity
    double *u_up;    // i + 1/2 velocity
    double *v_down;  // j - 1/2 velocity
    double *v_up;    // j + 1/2 velocity
    double *w_down;  // k - 1/2 velocity
    double *w_up;    // k - 1/2 velocity

    // FireParameter Pointers for normal solving (maaybe there is a better way to implement this)
    // kinda like a mesh now
    FireVoxel *i_down; // i - 1 param
    FireVoxel *i_up;   // i + 1 param
    FireVoxel *j_down; // j - 1 param
    FireVoxel *j_up;   // j + 1 param
    FireVoxel *k_down; // k - 1 param
    FireVoxel *k_up;   // k + 1 param

    Vector3D normal();
    Vector3D uf();
    Vector3D w(double s);   // implicit surface velocity
                            // not sure how to pass S efficiently b/c different structs
    void update_phi(double delta_t, double s);
    void update_temp();
};

struct FireParameters {
  FireParameters() {}
  FireParameters(double S) : S(S) {}
  ~FireParameters() {}

  // Fire Properties
  double S;   // reaction speed
};


struct Fire {
    Fire() {}
    ~Fire() {}

    // N^3 Voxel
    // paper stores implicit surface, temperature, density, pressure at each voxel center
    vector<FireVoxel *> map;

    // store separate map for just the points where phi == 0 for basic point rendering
    vector<FireVoxel *> implicit_surface;
    vector<FireVoxel *> fuel;
    vector<FireVoxel *> source;

    void build_map();
    void simulate(double delta_t, FireParameters *fp);
};
#endif /* FIRE_H */