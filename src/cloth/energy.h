#ifndef SILK_ENERGY_H
#define SILK_ENERGY_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/math.h"
#include "silk/world.h"

/* Element energies and their per-vertex derivatives. Every function is
 * pure: the solver, tests and the energy report share these exact kernels.
 * Gradients are dE/dx (the force is the negative). Hessian blocks are the
 * diagonal 3x3 block d2E/dx_k2 of the element, made positive semidefinite
 * where noted so every vertex solve is a descent step. */

/* Triangle smaller than this rest area, m^2, is rejected. A 1 mm right
 * triangle is 5e-7 m^2; this leaves five decades below it. */
#define SL_TRIANGLE_AREA_MIN 1.0e-12f

/* Computes the inverse rest matrix (a, b, c, d) and rest area of triangle
 * (x0, x1, x2) from its own plane. Returns false when the area is below
 * SL_TRIANGLE_AREA_MIN or the result is non-finite. */
bool sl_membrane_rest(sl_vec3 x0, sl_vec3 x1, sl_vec3 x2, float rest[4],
                      float *area);

/* StVK membrane: E = A (mu |G|^2 + lambda / 2 tr(G)^2), with Green strain
 * G = (F^T F - I) / 2. Joules. */
float sl_membrane_energy(const sl_vec3 x[3], const float rest[4], float area,
                         float mu, float lambda);

/* Gradient and PSD Hessian block of corner in [0, 3). The exact block is
 * A [ (w^T S w) I + (mu + lambda) u u^T + mu |w|^2 F F^T ] with
 * S = 2 mu G + lambda tr(G) I and u = F w; the first coefficient is clamped
 * at zero, which only drops curvature under compression. */
void sl_membrane_vertex(const sl_vec3 x[3], const float rest[4], float area,
                        float mu, float lambda, uint32_t corner,
                        sl_vec3 *gradient, sl_mat3 *hessian);

/* Quadratic isometric bending (Bergou et al., "A Quadratic Bending Model
 * for Inextensible Surfaces", SGP 2006) on the stencil x[0], x[1] (shared
 * edge) and x[2], x[3] (wings):
 *
 *     E = k / 2 * scale * |sum_j weight_j x_j|^2
 *
 * weight holds the cotangent sums of the rest stencil, which must be flat.
 * The weights sum to zero and annihilate the rest positions, so E vanishes
 * on every affine image of the rest stencil: rigid motion and in-plane
 * stretch never register as bending. scale is
 * 1 / (A0 + A1) of the two rest triangles; with it, E tends to the plate
 * energy k / 2 * integral (kappa1 + kappa2)^2 dA under refinement, so k is
 * the bending modulus. Pinned by tests/test_energy.c (cylinder). Returns
 * false for a degenerate stencil. */
bool sl_bend_rest(const sl_vec3 x[4], float weight[4], float *scale);

float sl_bend_energy(const sl_vec3 x[4], const float weight[4], float scale,
                     float stiffness);

/* Gradient of slot in [0, 4); the Hessian block is the constant
 * stiffness * scale * weight_slot^2 * I, returned as its scalar. */
void sl_bend_vertex(const sl_vec3 x[4], const float weight[4], float scale,
                    float stiffness, uint32_t slot, sl_vec3 *gradient,
                    float *hessian);

/* Signed distance, meters, from p to the collider surface (positive
 * outside) and the outward unit normal there. A point on a sphere center
 * or capsule axis gets an arbitrary but fixed unit normal. collider must be
 * validated. */
float sl_collider_distance(const sl_collider *collider, sl_vec3 p,
                           sl_vec3 *normal);

#endif /* SILK_ENERGY_H */
