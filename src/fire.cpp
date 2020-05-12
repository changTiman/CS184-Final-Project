#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>
#include <queue>

#include "fire.h"

using namespace std;

constexpr auto VOXEL_H = 0.05;									// arbitrary number, can change this [meters]
constexpr auto N = 30;											// also arbitrary;
const auto SOURCE = Vector3D(N/2 * VOXEL_H, 0, N/2 * VOXEL_H);	// source of fuel
constexpr auto FUEL_R = 5;										// radius of fuel source [points]
constexpr auto S = 0.5;											// S as per paper [m/s]

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
	if (phi_x != 0 || phi_y != 0 || phi_z != 0) {
		normal.normalize();
	}	
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
	Vector3D u = uf();
	Vector3D n = normal();
	return u + s * n;
}

vector<FireVoxel*> FireVoxel::get_neighbors() {
	vector<FireVoxel*> ret;
	ret.emplace_back(i_down);
	ret.emplace_back(i_up);
	ret.emplace_back(j_down);
	ret.emplace_back(j_up);
	ret.emplace_back(k_down);
	ret.emplace_back(k_up);

	return ret;
}

void FireVoxel::calculate_phi(double delta_t, FireParameters *fp) {
	// upwind differencing approach
	Vector3D w_vec = w(fp->S);
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

	conditioned = false;
  pending_phi = phi - delta_t * (w_vec.x * phi_x + w_vec.y * phi_y + w_vec.z * phi_z);
	/*if ((phi > 0 && prev_phi > 0 && prev_phi - phi > 0) || (phi < 0 && prev_phi < 0 && prev_phi - phi < 0)) {
		cout << "nice" << endl;
	}*/
}

void FireVoxel::update_phi() {
  prev_phi = phi;
  if (!fixed_phi) {
    phi = pending_phi;
  }
}

void FireVoxel::update_temp() {
	// naive temperature based on linear distance from source
	temp = (double) 200.0 - (position - SOURCE).norm() * 50;
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

				FireVoxel *fv = new FireVoxel(-1, 20, 1.3, 1, pos);


				// -1 because no fuel, 0 deg Celcius,
				// 1.3 kg/m^3 density, 1 atm
				// values are kinda made up for now cause units are hard			
				
				// fake a plane where phi == 0 for render testing
				/*if (j == N / 2) {
					implicit_surface.emplace_back(fv);
				}
				else if (j <= N / 2 && (SOURCE - Vector3D(x, 0, z)).norm() <= FUEL_R * VOXEL_H) {
					fv->phi = 1;
					fv->temp = 150;
					fuel.emplace_back(fv);
					fv->update_temp();
				}*/

//				fv->phi *= 1 + (((float) rand()) / RAND_MAX / 1000);

				map.emplace_back(fv);

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

	// initialize source in middle of j == 0 plane
	int source_bot = N / 2 - FUEL_R + 1;
	int source_top = N / 2 + FUEL_R;

	for (int i = source_bot; i < source_top; i++) {
		for (int k = source_bot; k < source_top; k++) {
			FireVoxel* curr = map[i * N * N + k];
			if ((SOURCE - curr->position).norm() <= FUEL_R * VOXEL_H) {
				curr->phi = 1;
				curr->temp = 200;
				curr->fixed_phi = true;
				*(curr->v_down) = 1.0;

				// test velocity field init
				//*(curr->u_down) = 2.0;

				*(curr->v_up) = 5.0;
				source.emplace_back(curr);
				fuel.emplace_back(curr);	// source is also fuel

				curr->j_up->phi = 0;
				implicit_surface.emplace_back(curr->j_up);
				//implicit_surface.emplace_back(curr);
			}
		}
	}
}

void Fire::simulate(double delta_t, FireParameters *fp, vector<Vector3D> external_accelerations) {
	// fuel propogation from implicit surface
	// ISSUE: the velocities have to be synced up with delta_t because we aren't tracking individual particles
	for (auto f : source) {
		double mass = VOXEL_H * VOXEL_H * VOXEL_H * f->rho;
		double acc = std::accumulate(external_accelerations.begin(), external_accelerations.end(), Vector3D()).y;
		double damping = 0.8;		// arbitrary damping value
		//Vector3D uf = f->uf();	// this averages down velocity too fast

		//bool s = std::find(source.begin(), source.end(), f) != source.end();
		
		double dist = damping * (*(f->v_down) * delta_t + 0.5 * acc * delta_t * delta_t);
		FireVoxel *curr = f;

		// update all voxels that will be passed through by a single fuel particle in delta_t
		for (int j = 0; j < floor(dist / VOXEL_H); j++) {
			double next_vel = (damping) * (*(curr->v_down) + acc * delta_t);
			*(curr->v_up) = next_vel;
			curr = curr->j_up;
		}
	}

	fuel.clear();
	implicit_surface.clear();
  // Calculate new phi values
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      for (int k = 0; k < N; k++) {
        FireVoxel* fv = map[i * N * N + j * N + k];
        fv->calculate_phi(delta_t, fp);

        double phi_eps = 0.1;
        if (abs(fv->phi) < phi_eps) {
          implicit_surface.emplace_back(fv);
        }
        else if (fv->phi > 0) {
          fuel.emplace_back(fv);
        }
      }
    }
  }
	// Update phi values
	for (int i = 0; i < N; i++) {
	  for (int j = 0; j < N; j++) {
	    for (int k = 0; k < N; k++) {
	      FireVoxel *fv = map[i * pow(N, 2) + j * N + k];
	      fv->update_phi();
	    }
	  }
	}
//  cout << "implicit: " << implicit_surface.size() << endl;
//  cout << "fuel: " << fuel.size() << endl;
}

void Fire::condition_phi() {
  // keep implicit surfadce well conditioned
  // |delta_phi| == 1
  queue<FireVoxel *> march;
  //queue<FireVoxel *> neighbors;

  for (auto f : implicit_surface) {
    //f->phi = 0;
    f->conditioned = true;
    f->set_num = 0;
    for (auto n : f->get_neighbors()) {
      if (!n->conditioned) {
        march.push(n);
      }
    }
  }

  while (!march.empty()) {
    FireVoxel* fv = march.front();
    march.pop();

    if (!fv->conditioned) {
      unsigned int min_set = UINT_MAX;
      FireVoxel* condition_fv;

      for (auto n : fv->get_neighbors()) {
        if (n->conditioned && n->set_num < min_set) {
          condition_fv = n;
          min_set = n->set_num;
        }
        else if (!n->conditioned) {
          march.push(n);
        }
      }

      // ensure |delta_phi| == 1 with min_set conditioned fvs
      fv->phi = (fv->phi < condition_fv->phi) ? condition_fv->phi - 1 : condition_fv->phi + 1;

      fv->set_num = condition_fv->set_num + 1;
      fv->conditioned = true;
    }
  }
}

void Fire::reset() {
  map.clear();
  implicit_surface.clear();
  fuel.clear();
  source.clear();
  cout<<"Reset Fire"<<endl;
  build_map();
}