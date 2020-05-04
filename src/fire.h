#ifndef FIRE_H
#define FIRE_H

#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"

using namespace CGL;
using namespace std;

struct FireParameters {
    FireParameters() {}
    FireParameters(double phi, double temp, double rho, double pres);
    ~FireParameters();

    // phi will define whether rho and p are for fuel or hot gaseous products
    double phi;         // implicit surface definition:  
                        // positive in the region of space filled with fuel,
                        // negative elsewhere, and zero at the reaction zone
    double prev_phi;    // phi at last time step
    double temp;        // temperature
    double rho;         // density
    double pres;        // pressure   

    // Velocity components
    double *u_down;  // i - 1/2 velocity
    double *u_up;    // i + 1/2 velocity
    double *v_down;  // j - 1/2 velocity
    double *v_up;    // j + 1/2 velocity
    double *w_down;  // k - 1/2 velocity
    double *w_up;    // k - 1/2 velocity

    // FireParameter Pointers for normal solving (maaybe there is a better way to implement this)
    // kinda like a mesh now
    FireParameters *i_down; // i - 1 param
    FireParameters *i_up;   // i + 1 param
    FireParameters *j_down; // j - 1 param
    FireParameters *j_up;   // j + 1 param
    FireParameters *k_down; // k - 1 param
    FireParameters *k_up;   // k + 1 param

    Vector3D normal();
    Vector3D uf();
    Vector3D w(double s);   // implicit surface velocity
                            // not sure how to pass S efficiently b/c different structs
    void update_phi(double delta_t, double s);
};

struct Fire {
    Fire();
    Fire(float S);
    ~Fire();

    // Fire Properties
    double S;   // reaction speed

    // N^3 Voxel
    // paper stores implicit surface, temperature, density, pressure at each voxel center
    vector<FireParameters *> map;

    void build_map();
};

#endif /* FIRE_H */