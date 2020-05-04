#include <iostream>
#include <math.h>
#include <vector>

#include "fire.h"

using namespace std;

#define VOXEL_H 0.1	// arbitrary number, can change this

FireParameters::FireParameters(double phi, double temp, double rho, double pres) {
	this->phi = phi;
	this->temp = temp;
	this->rho = rho;
	this->pres = pres;
}

Vector3D FireParameters::normal() {
	double phi_x = (i_up->phi - i_down->phi) / (2 * VOXEL_H);
	double phi_y = (j_up->phi - j_down->phi) / (2 * VOXEL_H);
	double phi_z = (k_up->phi - k_down->phi) / (2 * VOXEL_H);
	return Vector3D(phi_x, phi_y, phi_z);
}

Vector3D FireParameters::uf() {
	double u = (*u_down + *u_up) / 2;
	double v = (*v_down + *v_up) / 2;
	double w = (*w_down + *w_up) / 2;
	return Vector3D(u, v, w);
}

Vector3D FireParameters::w(double s) {
	return uf() + s * normal();
}

void FireParameters::update_phi(double delta_t, double s) {
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
	phi = phi_x + phi_y + phi_z;	// not really sure how this works out (phi_x notation confusing on the paper)
}