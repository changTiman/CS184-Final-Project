#ifndef FIRE_H
#define FIRE_H

#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"

using namespace CGL;
using namespace std;

struct FireParameters {
    FireParameters() {}
    FireParameters(int phi, float t, float rho, float p);
    ~FireParameters();

    // phi will define whether rho and p are for fuel or hot gaseous products
    int phi;        // implicit surface definition:  
                    // positive in the region of space filled with fuel,
                    // negative elsewhere, and zero at the reaction zone
    float t;        // temperature
    float rho;      // density
    float p;        // pressure    
};

struct Fire {
    Fire();
    Fire(float S);
    ~Fire();

    // Fire Properties
    Vector3D uf;    // 3D velocity of the fuel
    Vector3D w;     // implicit surface velocity
    float S;        // reaction speed

    // N^3 Voxel
    // paper stores implicit surface, temperature, density, pressure at each voxel center
    vector<FireParameters> map;
};

#endif /* FIRE_H */