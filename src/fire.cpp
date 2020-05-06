#include <iostream>
#include <math.h>
#include <vector>

#include "fire.h"

using namespace std;

constexpr auto VOXEL_H = 0.05;	// arbitrary number, can change this;
// i lowered it to 30 for speed
constexpr auto N = 30;		// also arbitrary;
const auto SOURCE = Vector3D(0, 0, 0);	// source of fuel

//FireVoxel::FireVoxel(double phi, double temp, double rho, double pres) {
//	this->phi = phi;
//	this->temp = temp;
//	this->rho = rho;
//	this->pres = pres;
//}

// normal n: normal to level set
Vector3D FireVoxel::normal() {
	double phi_x = (i_up->phi - i_down->phi) / (2 * VOXEL_H);
	double phi_y = (j_up->phi - j_down->phi) / (2 * VOXEL_H);
	double phi_z = (k_up->phi - k_down->phi) / (2 * VOXEL_H);
	auto normal = Vector3D(phi_x, phi_y, phi_z);
	normal.normalize();
	return normal;
}

// u_f: movement of fuel
Vector3D FireVoxel::uf() {
	double u = (*u_down + *u_up) / 2;
	double v = (*v_down + *v_up) / 2;
	double w = (*w_down + *w_up) / 2;
	return Vector3D(u, v, w);
}

// w: net movement of level set
Vector3D FireVoxel::w(double s) {
	return uf() + s * normal();
}

void FireVoxel::update_phi(double delta_t, double s) {
	// upwind differencing approach
	Vector3D w_vec = w(s);
	double phi_x, phi_y, phi_z;
	if (w_vec.x > 0) {
		phi_x = (phi - i_down->phi) / VOXEL_H;
	}
	else if (w_vec.x < 0) {
		phi_x = (i_up->phi - phi) / VOXEL_H;
	}
	else {
		phi_x = 0;
	}

	if (w_vec.y > 0) {
		phi_y = (phi - j_down->phi) / VOXEL_H;
	}
	else if (w_vec.y < 0) {
		phi_y = (j_up->phi - phi) / VOXEL_H;
	}
	else {
		phi_y = 0;
	}

	if (w_vec.z > 0) {
		phi_z = (phi - k_down->phi) / VOXEL_H;
	}
	else if (w_vec.z < 0) {
		phi_z = (k_up->phi - phi) / VOXEL_H;
	}
	else {
		phi_z = 0;
	}

	prev_phi = phi;
	phi = phi - delta_t * (w_vec.x * phi_x + w_vec.y * phi_y + w_vec.z * phi_z);
}

void FireVoxel::update_temp() {
	// naive temperature based on linear distance from source
	temp = (double) 200.0 - (position - SOURCE).norm() * 50;
	temp = 200 + (phi * 100);
}

void Fire::build_map() {
	// initialize FireVoxel
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			for (int k = 0; k < N; k++) {
				double x, y, z;
				x = i * VOXEL_H;
				y = j * VOXEL_H;
				z = k * VOXEL_H;

				Vector3D pos = Vector3D(x, y, z);

				// fake a plane where phi == 0 for render testing
				if (i + j + k == N / 2) {
					//implicit_surface.emplace_back(new FireVoxel(0, 100, 1.3, 1, pos));
				}
				else if (i + j + k < N / 2) {
					FireVoxel *fv = new FireVoxel(1, 150, 1.3, 1, pos);
					fuel.emplace_back(fv);
					fv->update_temp();
				}

				// -1 because no fuel, 0 deg Celcius,
				// 1.3 kg/m^3 density, 1 atm
				// values are kinda made up for now cause units are hard			
				double phi = i + j + k < N / 3 ? 1.1 : -1.1;
				phi += rand() / 100.0f;
				map.emplace_back(new FireVoxel(phi, 0, 1.3, 1, pos));
			}
		}
	}

	// handle FireVoxel pointers
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			for (int k = 0; k < N; k++) {
				// default to curr to avoid null pointer errors, this may lead
				// to some weird edge effects. should fix later.
				FireVoxel &curr = *map[i * N * N + j * N + k];
				curr.i_down = i == 0 ? &curr : map[(i - 1) * N * N + j * N + k];
				curr.i_up = i == N - 1 ? &curr : map[(i + 1) * N * N + j * N + k];
				curr.j_down = j == 0 ? &curr : map[i * N * N + (j - 1) * N + k];
				curr.j_up = j == N - 1 ? &curr : map[i * N * N + (j + 1) * N + k];
				curr.k_down = k == 0 ? &curr : map[i * N * N + j * N + k - 1];
				curr.k_up = k == N - 1 ? &curr : map[i * N * N + j * N + k + 1];
			}
		}
	}

	// handle velocity pointers
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			for (int k = 0; k < N; k++) {
				FireVoxel &curr = *map[i * N * N + j * N + k];
				if (i == N - 1) {
					curr.u_up = new double(0.0);
				}
				if (i == 0) {
					curr.u_down = new double(0.0);
				}
				else {
					auto u_d = new double(0.0);
					curr.u_down = u_d;
					map[(i - 1) * N * N + j * N + k]->u_up = u_d;
				}
				if (j == N - 1) {
					curr.v_up = new double(0.0);
				}
				if (j == 0) {
					curr.v_down = new double(0.0);
				}
				else {
					auto v_d = new double(0.0);
					curr.v_down = v_d;
					map[i * N * N + (j - 1) * N + k]->v_up = v_d;
				}
				if (k == N - 1) {
					curr.w_up = new double(0.0);
				}
				if (k == 0) {
					curr.w_down = new double(0.0);
				}
				else {
					auto w_d = new double(0.0);
					curr.w_down = w_d;
					map[i * N * N + j * N + k - 1]->w_up = w_d;
				}
			}
		}
	}
	// i==0 -> u_down = 
}

void Fire::simulate(double delta_t, FireParameters *fp) {
	implicit_surface.clear();
	fuel.clear();
	// need to add a way to propogate fuel velocities for this to work

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			for (int k = 0; k < N; k++) {
				FireVoxel* fv = map[i * N * N + j * N + k];
				fv->update_phi(delta_t, 0.1);

				if (fv->phi == 0) {
					implicit_surface.emplace_back(fv);
				}
				else if (fv->phi > 0) {
					fuel.emplace_back(fv);
				}
			}
		}
	}
}