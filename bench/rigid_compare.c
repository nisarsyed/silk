/* Head-to-head rigid-body solver comparison (research harness for issue
 * #125, not part of the library). Both solvers run on silk's rigid
 * collision pipeline (src/rigid/rigid.h): the same bodies, contacts detected
 * once per step with the same margin, and the same warm-start storage, so only
 * the solver differs.
 *
 *   soft-step   Catto's Soft Step (Solver2D; Box2D v3; Box3D): per substep,
 *               integrate velocities, solve soft contacts and joints with a
 *               position bias, integrate positions, then relax without the
 *               bias. Contact softness 30 Hz (capped at a quarter of the
 *               substep rate), damping ratio 10, push-out capped at 3 m/s;
 *               joints at twice the contact stiffness, damping ratio 2.
 *               Warm-started accumulated impulses, Coulomb friction on two
 *               tangents clamped to the friction circle.
 *   avbd        Augmented VBD (Giles et al. 2025): per iteration, a
 *               Gauss-Seidel sweep of 6x6 Newton steps per body on the
 *               incremental potential, hard constraints as augmented
 *               Lagrangian rows (penalty k, multiplier lambda), then the
 *               dual update; diagonal geometric stiffness, Baumgarte-like
 *               alpha, warm start of k and lambda with gamma.
 *     paper     beta = 10, alpha = 0.95, gamma = 0.99, k_start = 1
 *               (paper Table 2 and demo PENALTY_MIN).
 *     demo      beta = 1e4, alpha = 0.99, gamma = 0.999 (avbd-demo3d).
 *     scaled    k_start = m / h^2 and beta = 1000 m / h^2 per meter, with
 *               m the smaller dynamic mass of the row: the penalty is tied
 *               to the block's inertia, which is what the density issue
 *               (avbd-demo2d #11) and Newton's compliant ALM point at.
 *     alm       NVIDIA Newton's compliant ALM (newton/_src/solvers/vbd/
 *               rigid_vbd_kernels.py, Aug. 2026), hard contacts: fixed
 *               penalties rho = 1 / (W h^2) from each row's Delassus term W
 *               (no ramp), one 3-vector multiplier per contact whose
 *               tangent part is projected onto the Coulomb disk mu lambda_n,
 *               friction from slip with Newton's sliding Hessian metric,
 *               alpha = 0, multipliers retained by gamma = 0.999.
 *     alm-substep  alm with the budget spent on substeps of one
 *               iteration (contacts detected once per step), as Newton is
 *               typically run and as Soft Step spends its budget.
 *     alm-stab  alm with C0 stabilization alpha = 0.95 (Newton's option,
 *               its legacy default): hard contacts at alpha = 0 correct
 *               the whole penetration per substep, and rho grows as 1/h^2.
 *     alm-stab-sub  alm-stab with substeps.
 *   soft+joints  Soft Step for contacts and friction, with joints solved
 *               by a block-Newton pass after each substep's position
 *               update (rc_joint_pass) instead of as soft impulses.
 *   joints-first  the same joint pass, run before the contact solve on
 *               predicted positions x + h v and kept only as a velocity
 *               correction, so contacts have the last word.
 *     hybrid    the scaled AVBD penalty policy (start at the row's Delassus
 *               rho, ramp by beta = 1000 rho per meter, warm start, paper
 *               alpha and gamma) for normals and joints, with Newton's
 *               vector multiplier and Coulomb-disk projection for friction
 *               (tangent penalty fixed at its Delassus value; ramping it
 *               with the normal one made stacks explode at 2-8
 *               iterations).
 *     vfric     scaled, with friction moved out of the position solve to a
 *               velocity-level Coulomb pass afterwards, as silk's cloth
 *               does: per contact, the tangential relative velocity drops
 *               by at most mu |F_n| h W.
 *
 * Budgets are swept (substeps for soft-step, iterations for avbd) and each
 * run reports milliseconds per step, so solvers are compared at equal time
 * rather than equal iterations. Single-threaded on both sides.
 *
 * usage: sl_rigid_compare [--quick] [--scene NAME] */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "rigid.h"

#define RC_DT (1.0f / 60.0f)
#define RC_GRAVITY (-9.81f)
#define RC_MARGIN 0.02f
#define RC_FRICTION 0.5f
#define RC_JOINT_MAX 64u
#define RC_PLANE_MAX 2u
/* Block-Newton iterations of the joint pass per Soft Step substep. */
#define RC_JOINT_PASS_ITERATIONS 2u

static double rc_seconds(void)
{
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

/* ---------------------------------------------------------------------- */
/* Shared world                                                            */

typedef struct rc_joint {
    uint32_t a;
    uint32_t b;
    sl_vec3 local_a;
    sl_vec3 local_b;
    /* Soft Step accumulated impulse; avbd multipliers and penalties. */
    sl_vec3 impulse;
    float lambda[3];
    float k[3];
} rc_joint;

typedef struct rc_world {
    sl_body_set set;
    sl_rigid_plane planes[RC_PLANE_MAX];
    uint32_t plane_count;
    rc_joint joints[RC_JOINT_MAX];
    uint32_t joint_count;
    /* Previous-step velocities, for the avbd adaptive initial guess. */
    sl_vec3 *velocity_previous;
} rc_world;

static bool rc_world_init(rc_world *w, uint32_t bodies)
{
    memset(w, 0, sizeof(*w));
    w->velocity_previous = calloc(bodies, sizeof(sl_vec3));
    return w->velocity_previous != NULL &&
           sl_body_set_init(&w->set, bodies, 16u * bodies + 64u);
}

static void rc_world_destroy(rc_world *w)
{
    sl_body_set_destroy(&w->set);
    free(w->velocity_previous);
}

/* R diag(d) R^T = sum_k d_k c_k c_k^T over the rotation's columns. */
static sl_mat3 rc_inverse_inertia_world(const sl_body_set *s, uint32_t i)
{
    const sl_mat3 r = sl_quat_to_mat3(s->orientation[i]);
    const sl_vec3 d = s->inverse_inertia[i];
    sl_mat3 out = sl_mat3_scale(sl_mat3_outer(r.c0, r.c0), d.x);
    out = sl_mat3_mul_add(out, sl_mat3_outer(r.c1, r.c1), d.y);
    return sl_mat3_mul_add(out, sl_mat3_outer(r.c2, r.c2), d.z);
}

/* World anchor of a manifold side: body point, or the plane point. */
static sl_vec3 rc_anchor(const sl_body_set *s, uint32_t body, sl_vec3 local)
{
    if (body >= SL_RIGID_STATIC_BASE) {
        return local;
    }
    return sl_vec3_add(s->position[body],
                       sl_quat_rotate(s->orientation[body], local));
}

static float rc_inverse_mass(const sl_body_set *s, uint32_t body)
{
    return (body >= SL_RIGID_STATIC_BASE) ? 0.0f : s->inverse_mass[body];
}

/* Orthonormal tangents of a unit normal (Frisvad-style, deterministic). */
static void rc_tangents(sl_vec3 n, sl_vec3 *t1, sl_vec3 *t2)
{
    const sl_vec3 a = (sl_abs(n.x) < 0.57f) ? sl_vec3_make(1.0f, 0.0f, 0.0f)
                                            : sl_vec3_make(0.0f, 1.0f, 0.0f);
    *t1 = sl_vec3_normalize(sl_vec3_cross(n, a));
    *t2 = sl_vec3_cross(n, *t1);
}

/* ---------------------------------------------------------------------- */
/* Soft Step                                                               */

typedef struct rc_soft {
    float bias_rate;
    float mass_scale;
    float impulse_scale;
} rc_soft;

/* Soft constraint coefficients for frequency hertz, damping ratio zeta,
 * and step h (Catto, Soft Constraints / Solver2D). */
static rc_soft rc_make_soft(float hertz, float zeta, float h)
{
    rc_soft s = { 0.0f, 1.0f, 0.0f };
    if (hertz == 0.0f) {
        return s;
    }
    const float omega = 2.0f * SL_PI * hertz;
    const float a1 = 2.0f * zeta + h * omega;
    const float a2 = h * omega * a1;
    const float a3 = 1.0f / (1.0f + a2);
    s.bias_rate = omega / a1;
    s.mass_scale = a2 * a3;
    s.impulse_scale = a3;
    return s;
}

typedef struct rc_point {
    uint32_t a;
    uint32_t b;
    sl_vec3 local_a;
    sl_vec3 local_b;
    sl_vec3 ra; /* world offsets at step start */
    sl_vec3 rb;
    sl_vec3 n;
    sl_vec3 t1;
    sl_vec3 t2;
    float normal_mass;
    float tangent_mass[2];
    float *warm; /* manifold storage: normal, t1, t2 impulses */
} rc_point;

typedef struct rc_soft_state {
    sl_vec3 *saved_position; /* joints-first scratch */
    sl_quat *saved_orientation;
    rc_point *points;
    uint32_t point_count;
    sl_mat3 *inverse_inertia; /* world, per body */
} rc_soft_state;

static float rc_effective_mass(const rc_soft_state *st, const sl_body_set *s,
                               uint32_t a, uint32_t b, sl_vec3 ra, sl_vec3 rb,
                               sl_vec3 d)
{
    float k = rc_inverse_mass(s, a) + rc_inverse_mass(s, b);
    if (a < SL_RIGID_STATIC_BASE && s->inverse_mass[a] > 0.0f) {
        const sl_vec3 c = sl_vec3_cross(ra, d);
        k += sl_vec3_dot(c, sl_mat3_mul_vec3(st->inverse_inertia[a], c));
    }
    if (b < SL_RIGID_STATIC_BASE && s->inverse_mass[b] > 0.0f) {
        const sl_vec3 c = sl_vec3_cross(rb, d);
        k += sl_vec3_dot(c, sl_mat3_mul_vec3(st->inverse_inertia[b], c));
    }
    return (k > 0.0f) ? 1.0f / k : 0.0f;
}

static sl_vec3 rc_point_velocity(const sl_body_set *s, uint32_t body, sl_vec3 r)
{
    if (body >= SL_RIGID_STATIC_BASE) {
        return sl_vec3_zero();
    }
    return sl_vec3_add(s->velocity[body],
                       sl_vec3_cross(s->angular_velocity[body], r));
}

static void rc_apply(rc_soft_state *st, sl_body_set *s, uint32_t body,
                     sl_vec3 r, sl_vec3 impulse)
{
    if (body >= SL_RIGID_STATIC_BASE || s->inverse_mass[body] == 0.0f) {
        return;
    }
    s->velocity[body] =
        sl_vec3_mul_add(s->velocity[body], impulse, s->inverse_mass[body]);
    s->angular_velocity[body] = sl_vec3_add(
        s->angular_velocity[body],
        sl_mat3_mul_vec3(st->inverse_inertia[body], sl_vec3_cross(r, impulse)));
}

static void rc_soft_contacts(rc_soft_state *st, sl_body_set *s, float h,
                             rc_soft soft, bool use_bias, bool reverse)
{
    const float max_push = 3.0f;
    for (uint32_t k = 0; k < st->point_count; k++) {
        rc_point *p = &st->points[reverse ? st->point_count - 1u - k : k];
        const sl_vec3 pa = rc_anchor(s, p->a, p->local_a);
        const sl_vec3 pb = rc_anchor(s, p->b, p->local_b);
        const float separation = sl_vec3_dot(sl_vec3_sub(pb, pa), p->n);
        float bias = 0.0f;
        float mass_scale = 1.0f;
        float impulse_scale = 0.0f;
        if (separation > 0.0f) {
            bias = separation / h; /* speculative */
        } else if (use_bias) {
            bias = sl_max(soft.bias_rate * separation, -max_push);
            mass_scale = soft.mass_scale;
            impulse_scale = soft.impulse_scale;
        }
        const sl_vec3 dv = sl_vec3_sub(rc_point_velocity(s, p->b, p->rb),
                                       rc_point_velocity(s, p->a, p->ra));
        const float vn = sl_vec3_dot(dv, p->n);
        float impulse = -p->normal_mass * mass_scale * (vn + bias) -
                        impulse_scale * p->warm[0];
        const float total = sl_max(p->warm[0] + impulse, 0.0f);
        impulse = total - p->warm[0];
        p->warm[0] = total;
        const sl_vec3 pn = sl_vec3_scale(p->n, impulse);
        rc_apply(st, s, p->a, p->ra, sl_vec3_neg(pn));
        rc_apply(st, s, p->b, p->rb, pn);

        /* Friction on two tangents, clamped to the friction circle. */
        const sl_vec3 dv2 = sl_vec3_sub(rc_point_velocity(s, p->b, p->rb),
                                        rc_point_velocity(s, p->a, p->ra));
        const float l1 =
            p->warm[1] - p->tangent_mass[0] * sl_vec3_dot(dv2, p->t1);
        const float l2 =
            p->warm[2] - p->tangent_mass[1] * sl_vec3_dot(dv2, p->t2);
        const float limit = RC_FRICTION * p->warm[0];
        const float length = sqrtf(l1 * l1 + l2 * l2);
        const float scale =
            (length > limit && length > 0.0f) ? limit / length : 1.0f;
        const float n1 = l1 * scale;
        const float n2 = l2 * scale;
        const sl_vec3 pt = sl_vec3_add(sl_vec3_scale(p->t1, n1 - p->warm[1]),
                                       sl_vec3_scale(p->t2, n2 - p->warm[2]));
        p->warm[1] = n1;
        p->warm[2] = n2;
        rc_apply(st, s, p->a, p->ra, sl_vec3_neg(pt));
        rc_apply(st, s, p->b, p->rb, pt);
    }
}

/* -[r]x M [r]x, the angular part of a point constraint's mass matrix. */
static sl_mat3 rc_skew_sandwich(sl_vec3 r, sl_mat3 m)
{
    sl_mat3 out;
    const sl_vec3 e[3] = { sl_vec3_make(1, 0, 0), sl_vec3_make(0, 1, 0),
                           sl_vec3_make(0, 0, 1) };
    sl_vec3 cols[3];
    for (uint32_t k = 0; k < 3u; k++) {
        /* column k of -[r]x M [r]x = r x (M (e_k x r)) */
        cols[k] = sl_vec3_cross(r, sl_mat3_mul_vec3(m, sl_vec3_cross(e[k], r)));
    }
    out.c0 = cols[0];
    out.c1 = cols[1];
    out.c2 = cols[2];
    return out;
}

static void rc_soft_joints(rc_soft_state *st, rc_world *w, float h,
                           rc_soft soft, bool use_bias)
{
    (void)h;
    sl_body_set *s = &w->set;
    for (uint32_t k = 0; k < w->joint_count; k++) {
        rc_joint *j = &w->joints[k];
        const sl_vec3 ra = sl_quat_rotate(s->orientation[j->a], j->local_a);
        const sl_vec3 rb = sl_quat_rotate(s->orientation[j->b], j->local_b);
        const sl_vec3 c = sl_vec3_sub(sl_vec3_add(s->position[j->b], rb),
                                      sl_vec3_add(s->position[j->a], ra));
        sl_mat3 kmat =
            sl_mat3_diagonal(s->inverse_mass[j->a] + s->inverse_mass[j->b]);
        kmat =
            sl_mat3_add(kmat, rc_skew_sandwich(ra, st->inverse_inertia[j->a]));
        kmat =
            sl_mat3_add(kmat, rc_skew_sandwich(rb, st->inverse_inertia[j->b]));
        const sl_vec3 cdot = sl_vec3_sub(rc_point_velocity(s, j->b, rb),
                                         rc_point_velocity(s, j->a, ra));
        sl_vec3 rhs = cdot;
        float mass_scale = 1.0f;
        float impulse_scale = 0.0f;
        if (use_bias) {
            rhs = sl_vec3_mul_add(cdot, c, soft.bias_rate);
            mass_scale = soft.mass_scale;
            impulse_scale = soft.impulse_scale;
        }
        sl_vec3 solved;
        if (!sl_mat3_solve(kmat, rhs, 0.0f, &solved)) {
            continue;
        }
        const sl_vec3 impulse =
            sl_vec3_sub(sl_vec3_scale(solved, -mass_scale),
                        sl_vec3_scale(j->impulse, impulse_scale));
        j->impulse = sl_vec3_add(j->impulse, impulse);
        rc_apply(st, s, j->a, ra, sl_vec3_neg(impulse));
        rc_apply(st, s, j->b, rb, impulse);
    }
}

/* Applies the accumulated impulses (warm start). Box2D v3 does this at the
 * start of every substep: an accumulated impulse is one substep's worth.
 * Applying it once per step left every later substep under-supported (a
 * 10-box stack sank 26 mm and a 30-box stack fell). */
static void rc_soft_warm(rc_soft_state *st, rc_world *w)
{
    sl_body_set *s = &w->set;
    for (uint32_t k = 0; k < st->point_count; k++) {
        const rc_point *p = &st->points[k];
        const sl_vec3 impulse =
            sl_vec3_add(sl_vec3_scale(p->n, p->warm[0]),
                        sl_vec3_add(sl_vec3_scale(p->t1, p->warm[1]),
                                    sl_vec3_scale(p->t2, p->warm[2])));
        rc_apply(st, s, p->a, p->ra, sl_vec3_neg(impulse));
        rc_apply(st, s, p->b, p->rb, impulse);
    }
    for (uint32_t k = 0; k < w->joint_count; k++) {
        const rc_joint *j = &w->joints[k];
        const sl_vec3 ra = sl_quat_rotate(s->orientation[j->a], j->local_a);
        const sl_vec3 rb = sl_quat_rotate(s->orientation[j->b], j->local_b);
        rc_apply(st, s, j->a, ra, sl_vec3_neg(j->impulse));
        rc_apply(st, s, j->b, rb, j->impulse);
    }
}

typedef struct rc_avbd_state rc_avbd_state;
static void rc_joint_pass(rc_world *w, rc_avbd_state *st, float h,
                          uint32_t iterations);

/* Soft Step. With joint_pass non-NULL, joints are not solved as soft
 * impulses: after each substep's position update a block-Newton pass
 * enforces them (rc_joint_pass), before the relax. */
static void rc_soft_step(rc_world *w, uint32_t substeps, bool reverse,
                         rc_soft_state *st, rc_avbd_state *joint_pass,
                         bool joints_first)
{
    sl_body_set *s = &w->set;
    const float h = RC_DT / (float)substeps;
    const float contact_hertz = sl_min(30.0f, 0.25f / h);
    const rc_soft contact_soft = rc_make_soft(contact_hertz, 10.0f, h);
    const rc_soft joint_soft = rc_make_soft(2.0f * contact_hertz, 2.0f, h);
    for (uint32_t i = 0; i < s->count; i++) {
        st->inverse_inertia[i] = rc_inverse_inertia_world(s, i);
    }
    /* Prepare contact points with warm starting. */
    st->point_count = 0;
    for (uint32_t m = 0; m < s->manifold_count; m++) {
        sl_manifold *mf = &s->manifolds[m];
        sl_vec3 t1;
        sl_vec3 t2;
        rc_tangents(mf->normal, &t1, &t2);
        for (uint32_t q = 0; q < mf->point_count; q++) {
            rc_point *p = &st->points[st->point_count++];
            p->a = mf->body_a;
            p->b = mf->body_b;
            p->local_a = mf->points[q].local_a;
            p->local_b = mf->points[q].local_b;
            p->ra =
                sl_vec3_sub(rc_anchor(s, p->a, p->local_a), s->position[p->a]);
            p->rb = (p->b >= SL_RIGID_STATIC_BASE)
                        ? sl_vec3_zero()
                        : sl_vec3_sub(rc_anchor(s, p->b, p->local_b),
                                      s->position[p->b]);
            p->n = mf->normal;
            p->t1 = t1;
            p->t2 = t2;
            p->normal_mass =
                rc_effective_mass(st, s, p->a, p->b, p->ra, p->rb, p->n);
            p->tangent_mass[0] =
                rc_effective_mass(st, s, p->a, p->b, p->ra, p->rb, t1);
            p->tangent_mass[1] =
                rc_effective_mass(st, s, p->a, p->b, p->ra, p->rb, t2);
            p->warm = mf->points[q].warm;
        }
    }
    for (uint32_t sub = 0; sub < substeps; sub++) {
        for (uint32_t i = 0; i < s->count; i++) {
            if (s->inverse_mass[i] > 0.0f) {
                s->velocity[i].y += h * RC_GRAVITY;
            }
        }
        rc_soft_warm(st, w);
        if (joint_pass == NULL) {
            rc_soft_joints(st, w, h, joint_soft, true);
        } else if (joints_first) {
            /* Joint pass on the predicted positions x + h v, as a velocity
             * correction; positions are restored, so the contact solve
             * that follows has the last word on penetration. */
            for (uint32_t i = 0; i < s->count; i++) {
                st->saved_position[i] = s->position[i];
                st->saved_orientation[i] = s->orientation[i];
                if (s->inverse_mass[i] == 0.0f) {
                    continue;
                }
                s->position[i] =
                    sl_vec3_mul_add(s->position[i], s->velocity[i], h);
                const sl_vec3 omega = s->angular_velocity[i];
                const sl_quat q = s->orientation[i];
                const sl_quat dq = sl_quat_mul(
                    sl_quat_make(omega.x, omega.y, omega.z, 0.0f), q);
                s->orientation[i] = sl_quat_normalize(
                    sl_quat_make(q.x + 0.5f * h * dq.x, q.y + 0.5f * h * dq.y,
                                 q.z + 0.5f * h * dq.z, q.w + 0.5f * h * dq.w));
            }
            rc_joint_pass(w, joint_pass, h, RC_JOINT_PASS_ITERATIONS);
            for (uint32_t i = 0; i < s->count; i++) {
                s->position[i] = st->saved_position[i];
                s->orientation[i] = st->saved_orientation[i];
            }
        }
        rc_soft_contacts(st, s, h, contact_soft, true, reverse);
        for (uint32_t i = 0; i < s->count; i++) {
            if (s->inverse_mass[i] == 0.0f) {
                continue;
            }
            s->position[i] = sl_vec3_mul_add(s->position[i], s->velocity[i], h);
            const sl_vec3 omega = s->angular_velocity[i];
            const sl_quat q = s->orientation[i];
            const sl_quat dq =
                sl_quat_mul(sl_quat_make(omega.x, omega.y, omega.z, 0.0f), q);
            s->orientation[i] = sl_quat_normalize(
                sl_quat_make(q.x + 0.5f * h * dq.x, q.y + 0.5f * h * dq.y,
                             q.z + 0.5f * h * dq.z, q.w + 0.5f * h * dq.w));
            st->inverse_inertia[i] = rc_inverse_inertia_world(s, i);
        }
        if (joint_pass != NULL && !joints_first) {
            rc_joint_pass(w, joint_pass, h, RC_JOINT_PASS_ITERATIONS);
            for (uint32_t i = 0; i < s->count; i++) {
                st->inverse_inertia[i] = rc_inverse_inertia_world(s, i);
            }
        } else if (joint_pass == NULL) {
            rc_soft_joints(st, w, h, joint_soft, false);
        }
        rc_soft_contacts(st, s, h, contact_soft, false, reverse);
    }
}

/* ---------------------------------------------------------------------- */
/* AVBD                                                                    */

typedef enum rc_avbd_kind {
    RC_AVBD_PAPER = 0,
    RC_AVBD_DEMO = 1,
    RC_AVBD_SCALED = 2,
    RC_AVBD_SCALED_VF = 3,
    RC_AVBD_ALM = 4,
    RC_AVBD_ALM_SUB = 5,
    RC_AVBD_ALM_STAB = 6,
    RC_AVBD_ALM_STAB_SUB = 7,
    RC_AVBD_HYBRID = 8
} rc_avbd_kind;

typedef struct rc_avbd_params {
    float beta;
    float alpha;
    float gamma;
    bool scaled;
    bool velocity_friction; /* friction after the solve, not as rows */
    bool alm; /* Newton's compliant ALM with projected Coulomb friction */
    bool substepped; /* budget = substeps of one iteration */
} rc_avbd_params;

static rc_avbd_params rc_avbd_params_of(rc_avbd_kind kind)
{
    rc_avbd_params p = { 10.0f, 0.95f, 0.99f, false, false, false, false };
    if (kind == RC_AVBD_DEMO) {
        p.beta = 1.0e4f;
        p.alpha = 0.99f;
        p.gamma = 0.999f;
    } else if (kind == RC_AVBD_SCALED || kind == RC_AVBD_SCALED_VF) {
        p.beta = 1000.0f; /* times m / h^2, per meter */
        p.scaled = true;
        p.velocity_friction = (kind == RC_AVBD_SCALED_VF);
    } else if (kind == RC_AVBD_HYBRID) {
        p.beta = 1000.0f; /* times rho_start, per meter */
        p.scaled = true;
        p.alm = true;
    } else if (kind == RC_AVBD_ALM || kind == RC_AVBD_ALM_SUB ||
               kind == RC_AVBD_ALM_STAB || kind == RC_AVBD_ALM_STAB_SUB) {
        /* Newton defaults under compliant ALM: no C0 stabilization,
         * multipliers retained by gamma, fixed penalties (no ramp). */
        p.beta = 0.0f;
        p.alpha = 0.0f;
        p.gamma = 0.999f;
        p.alm = true;
        p.substepped =
            (kind == RC_AVBD_ALM_SUB || kind == RC_AVBD_ALM_STAB_SUB);
        if (kind == RC_AVBD_ALM_STAB || kind == RC_AVBD_ALM_STAB_SUB) {
            p.alpha = 0.95f; /* Newton's option, its legacy default */
        }
    }
    return p;
}

#define RC_PENALTY_MIN 1.0f
#define RC_PENALTY_MAX 1.0e10f

typedef enum rc_row_kind {
    RC_ROW_NORMAL = 0,
    RC_ROW_FRICTION = 1,
    RC_ROW_JOINT = 2
} rc_row_kind;

typedef struct rc_row {
    uint32_t a;
    uint32_t b;
    sl_vec3 local_a;
    sl_vec3 local_b;
    sl_vec3 d;       /* world direction, fixed for the step */
    float c0;        /* C at the step start (normal and joint rows) */
    sl_vec3 start;   /* pb - pa at the step start (friction rows) */
    uint32_t normal; /* friction rows: index of their normal row */
    rc_row_kind kind;
    float k;
    float lambda;
    float k_start;
    float beta;
    float *warm_lambda; /* persistent storage, or NULL */
    float *warm_k;
} rc_row;

/* One contact point under compliant ALM: a 3-vector multiplier (normal
 * component >= 0, tangent within the Coulomb disk) and fixed penalties
 * from the point's effective mass along the normal and tangents. */
typedef struct rc_alm_contact {
    uint32_t a;
    uint32_t b;
    sl_vec3 local_a;
    sl_vec3 local_b;
    sl_vec3 n;
    sl_vec3 pa0; /* anchors at the step start, for slip */
    sl_vec3 pb0;
    float rho_n;
    float rho_t;
    float beta;     /* penalty ramp per meter of penetration (0: fixed) */
    float alpha;    /* C0 stabilization: penetration minus alpha depth0 */
    float depth0;   /* penetration at the step start, >= 0 */
    sl_vec3 lambda; /* world; force on b is +, on a - */
    float *warm;    /* manifold storage: lambda in warm[0..2] */
} rc_alm_contact;

typedef struct rc_avbd_state {
    rc_alm_contact *contacts;
    uint32_t contact_count;
    uint32_t *body_contact_offset; /* bodies + 1 */
    uint32_t *body_contact;
    rc_row *rows;
    uint32_t row_count;
    uint32_t *body_row_offset; /* bodies + 1 */
    uint32_t *body_row_fill;   /* bodies + 1, scratch */
    uint32_t *body_row;
    sl_vec3 *x0; /* positions and orientations at the step start */
    sl_quat *q0;
    sl_vec3 *x_inertial;
    sl_quat *q_inertial;
} rc_avbd_state;

static float rc_row_c(const sl_body_set *s, const rc_row *r, float alpha)
{
    const sl_vec3 diff = sl_vec3_sub(rc_anchor(s, r->b, r->local_b),
                                     rc_anchor(s, r->a, r->local_a));
    if (r->kind == RC_ROW_FRICTION) {
        return sl_vec3_dot(sl_vec3_sub(diff, r->start), r->d);
    }
    /* Hard constraints correct only (1 - alpha) of the step-start error
     * (paper Eq. 18); a positive speculative gap is kept likewise. */
    return sl_vec3_dot(diff, r->d) - alpha * r->c0;
}

/* Force on the row, clamped to its bounds. */
static float rc_row_force(const sl_body_set *s, const rc_row *rows,
                          const rc_row *r, float alpha, float *c_out)
{
    const float c = rc_row_c(s, r, alpha);
    *c_out = c;
    const float f = r->k * c + r->lambda;
    if (r->kind == RC_ROW_NORMAL) {
        return sl_min(f, 0.0f);
    }
    if (r->kind == RC_ROW_FRICTION) {
        const rc_row *n = &rows[r->normal];
        float cn;
        const float fn = rc_row_force(s, rows, n, alpha, &cn);
        const float limit = RC_FRICTION * sl_abs(fn);
        return sl_clamp(f, -limit, limit);
    }
    return f;
}

/* 6x6 SPD solve by Cholesky (LDL^T would do equally); h row-major. */
static bool rc_solve6(float h[6][6], const float f[6], float x[6])
{
    float l[6][6];
    memset(l, 0, sizeof(l));
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = h[i][j];
            for (int k = 0; k < j; k++) {
                sum -= l[i][k] * l[j][k];
            }
            if (i == j) {
                if (!(sum > 0.0f)) {
                    return false;
                }
                l[i][i] = sqrtf(sum);
            } else {
                l[i][j] = sum / l[j][j];
            }
        }
    }
    float y[6];
    for (int i = 0; i < 6; i++) {
        float sum = f[i];
        for (int k = 0; k < i; k++) {
            sum -= l[i][k] * y[k];
        }
        y[i] = sum / l[i][i];
    }
    for (int i = 5; i >= 0; i--) {
        float sum = y[i];
        for (int k = i + 1; k < 6; k++) {
            sum -= l[k][i] * x[k];
        }
        x[i] = sum / l[i][i];
    }
    return true;
}

/* ---- Compliant ALM contacts (Newton, rigid_vbd_kernels.py) ---------- */

/* Inverse effective mass of a point on a body along unit d. */
static float rc_delassus(const sl_body_set *s, uint32_t body, sl_vec3 r,
                         sl_vec3 d)
{
    if (body >= SL_RIGID_STATIC_BASE || s->inverse_mass[body] == 0.0f) {
        return 0.0f;
    }
    const sl_vec3 x = sl_vec3_cross(r, d);
    return s->inverse_mass[body] +
           sl_vec3_dot(x,
                       sl_mat3_mul_vec3(rc_inverse_inertia_world(s, body), x));
}

static sl_vec3 rc_lever(const sl_body_set *s, uint32_t body, sl_vec3 p)
{
    return (body >= SL_RIGID_STATIC_BASE) ? sl_vec3_zero()
                                          : sl_vec3_sub(p, s->position[body]);
}

/* Linearly regularized 1/u below eps (Newton's _regularized_coulomb_scale). */
static float rc_coulomb_scale(float u, float eps)
{
    if (u > eps && u > 0.0f) {
        return 1.0f / u;
    }
    return (eps > 0.0f) ? (-u / eps + 2.0f) / eps : 0.0f;
}

/* Projects v onto the disk of radius limit. */
static sl_vec3 rc_project_disk(sl_vec3 v, float limit)
{
    const float length = sl_vec3_length(v);
    if (!(limit > 0.0f)) {
        return sl_vec3_zero();
    }
    return (length > limit) ? sl_vec3_scale(v, limit / length) : v;
}

/* Penetration with C0 stabilization (Newton's C_stab = C - alpha C0),
 * bypassed once separated so support releases fully. */
static float rc_alm_depth(const sl_body_set *s, const rc_alm_contact *c)
{
    const sl_vec3 pa = rc_anchor(s, c->a, c->local_a);
    const sl_vec3 pb = rc_anchor(s, c->b, c->local_b);
    const float depth = -sl_vec3_dot(c->n, sl_vec3_sub(pb, pa));
    return (depth > 0.0f) ? depth - c->alpha * c->depth0 : depth;
}

/* Slip of b relative to a at the contact since the step start. */
static sl_vec3 rc_alm_slip(const sl_body_set *s, const rc_alm_contact *c)
{
    const sl_vec3 u =
        sl_vec3_sub(sl_vec3_sub(rc_anchor(s, c->b, c->local_b), c->pb0),
                    sl_vec3_sub(rc_anchor(s, c->a, c->local_a), c->pa0));
    return sl_vec3_mul_add(u, c->n, -sl_vec3_dot(u, c->n));
}

/* Adds contact c's force and Hessian to body i's block (Newton's
 * evaluate_rigid_contact_from_collision under compliant ALM, hard). */
static void rc_alm_contact_block(const sl_body_set *s, const rc_alm_contact *c,
                                 uint32_t i, float h, float f[6],
                                 float hm[6][6])
{
    const sl_vec3 pa = rc_anchor(s, c->a, c->local_a);
    const sl_vec3 pb = rc_anchor(s, c->b, c->local_b);
    const float depth = rc_alm_depth(s, c);
    const float lam_n = sl_vec3_dot(c->lambda, c->n);
    if (depth <= 1.0e-9f && lam_n <= 0.0f) {
        return;
    }
    const float fn = sl_max(c->rho_n * depth + lam_n, 0.0f);
    if (fn == 0.0f) {
        return; /* separated contacts release their Hessian */
    }
    const sl_vec3 n = c->n;
    float k[3][3];
    const float nv[3] = { n.x, n.y, n.z };
    for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) {
            k[a][b] = c->rho_n * nv[a] * nv[b];
        }
    }
    const sl_vec3 ut = rc_alm_slip(s, c);
    const sl_vec3 lam_t = sl_vec3_mul_add(c->lambda, n, -lam_n);
    sl_vec3 ft = sl_vec3_add(sl_vec3_scale(ut, -c->rho_t), lam_t);
    const float ft_len = sl_vec3_length(ft);
    const float cone = RC_FRICTION * fn;
    if (ft_len > cone && ft_len > 0.0f) {
        /* Sliding: the projection's stiffness across the slip, plus a
         * secant metric along it. */
        const sl_vec3 t = sl_vec3_scale(ft, 1.0f / ft_len);
        const float tv[3] = { t.x, t.y, t.z };
        const float across = (cone / ft_len) * c->rho_t;
        const float along =
            cone * rc_coulomb_scale(sl_vec3_length(ut), 0.01f * h);
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                const float id = (a == b) ? 1.0f : 0.0f;
                k[a][b] += across * (id - nv[a] * nv[b] - tv[a] * tv[b]) +
                           along * tv[a] * tv[b];
            }
        }
        ft = sl_vec3_scale(ft, cone / ft_len);
    } else {
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                const float id = (a == b) ? 1.0f : 0.0f;
                k[a][b] += c->rho_t * (id - nv[a] * nv[b]);
            }
        }
    }
    const bool is_b = (c->b == i);
    const sl_vec3 total = sl_vec3_mul_add(ft, n, fn); /* on b */
    const sl_vec3 force = is_b ? total : sl_vec3_neg(total);
    const sl_vec3 r = rc_lever(s, i, is_b ? pb : pa);
    const sl_vec3 torque = sl_vec3_cross(r, force);
    f[0] += force.x;
    f[1] += force.y;
    f[2] += force.z;
    f[3] += torque.x;
    f[4] += torque.y;
    f[5] += torque.z;
    /* H += J^T K J with J = [I, -S], S = skew(r):
     * [K, -K S; S K, -S K S]. */
    const float sk[3][3] = { { 0.0f, -r.z, r.y },
                             { r.z, 0.0f, -r.x },
                             { -r.y, r.x, 0.0f } };
    float ks[3][3];
    float sks[3][3];
    for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) {
            float sum = 0.0f;
            for (int m = 0; m < 3; m++) {
                sum += k[a][m] * sk[m][b];
            }
            ks[a][b] = sum;
        }
    }
    for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) {
            float sum = 0.0f;
            for (int m = 0; m < 3; m++) {
                sum += sk[a][m] * ks[m][b];
            }
            sks[a][b] = sum;
        }
    }
    for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) {
            hm[a][b] += k[a][b];
            hm[a][3 + b] -= ks[a][b];
            hm[3 + a][b] -= ks[b][a];
            hm[3 + a][3 + b] -= sks[a][b];
        }
    }
}

/* Dual step (Newton's _compliant_contact_dual_step, hard contact): the
 * normal multiplier ascends and stays >= 0, then the tangent trial is
 * projected into the Coulomb disk of the new normal multiplier. */
static void rc_alm_contact_dual(const sl_body_set *s, rc_alm_contact *c)
{
    const float depth = rc_alm_depth(s, c);
    const float lam_n = sl_vec3_dot(c->lambda, c->n);
    const sl_vec3 lam_t = sl_vec3_mul_add(c->lambda, c->n, -lam_n);
    const float lam_n_new = sl_max(lam_n + c->rho_n * depth, 0.0f);
    if (c->beta > 0.0f && lam_n_new > 0.0f) {
        /* AVBD's ramp (paper Eq. 12) on both penalties. */
        c->rho_n = sl_min(RC_PENALTY_MAX, c->rho_n + c->beta * sl_abs(depth));
    }
    const sl_vec3 trial =
        sl_vec3_add(lam_t, sl_vec3_scale(rc_alm_slip(s, c), -c->rho_t));
    c->lambda = sl_vec3_mul_add(rc_project_disk(trial, RC_FRICTION * lam_n_new),
                                c->n, lam_n_new);
}

static void rc_avbd_body(rc_world *w, rc_avbd_state *st, uint32_t i, float h,
                         const rc_avbd_params *prm)
{
    sl_body_set *s = &w->set;
    const float mass = 1.0f / s->inverse_mass[i];
    const sl_vec3 inertia = sl_vec3_make(1.0f / s->inverse_inertia[i].x,
                                         1.0f / s->inverse_inertia[i].y,
                                         1.0f / s->inverse_inertia[i].z);
    const sl_mat3 r = sl_quat_to_mat3(s->orientation[i]);
    float hm[6][6];
    float f[6];
    memset(hm, 0, sizeof(hm));
    const float inv_h2 = 1.0f / (h * h);
    for (int k = 0; k < 3; k++) {
        hm[k][k] = mass * inv_h2;
    }
    /* World inertia R diag(I) R^T. */
    const sl_vec3 col[3] = { r.c0, r.c1, r.c2 };
    const float ix[3] = { inertia.x, inertia.y, inertia.z };
    for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                const float ca =
                    (a == 0) ? col[k].x : ((a == 1) ? col[k].y : col[k].z);
                const float cb =
                    (b == 0) ? col[k].x : ((b == 1) ? col[k].y : col[k].z);
                sum += ca * ix[k] * cb;
            }
            hm[3 + a][3 + b] = sum * inv_h2;
        }
    }
    const sl_vec3 dx = sl_vec3_sub(s->position[i], st->x_inertial[i]);
    /* Rotation difference 2 vec(q q_inertial*), sign-consistent. */
    sl_quat dq =
        sl_quat_mul(s->orientation[i], sl_quat_conjugate(st->q_inertial[i]));
    if (dq.w < 0.0f) {
        dq = sl_quat_make(-dq.x, -dq.y, -dq.z, -dq.w);
    }
    const float dtheta[3] = { 2.0f * dq.x, 2.0f * dq.y, 2.0f * dq.z };
    f[0] = -mass * inv_h2 * dx.x;
    f[1] = -mass * inv_h2 * dx.y;
    f[2] = -mass * inv_h2 * dx.z;
    for (int a = 0; a < 3; a++) {
        float sum = 0.0f;
        for (int b = 0; b < 3; b++) {
            sum += hm[3 + a][3 + b] * dtheta[b];
        }
        f[3 + a] = -sum;
    }
    for (uint32_t ref = st->body_row_offset[i];
         ref < st->body_row_offset[i + 1u]; ref++) {
        const rc_row *row = &st->rows[st->body_row[ref]];
        float c;
        const float force = rc_row_force(s, st->rows, row, prm->alpha, &c);
        const bool is_b = (row->b == i);
        const sl_vec3 rr =
            sl_vec3_sub(rc_anchor(s, i, is_b ? row->local_b : row->local_a),
                        s->position[i]);
        const float sign = is_b ? 1.0f : -1.0f;
        const sl_vec3 ang = sl_vec3_cross(rr, row->d);
        const float j[6] = { sign * row->d.x, sign * row->d.y, sign * row->d.z,
                             sign * ang.x,    sign * ang.y,    sign * ang.z };
        /* The unclamped k in the Hessian (the paper's simple solution).
         * Stiffness rescaling of clamped rows (paper Eq. 14) was tried:
         * on every row it made stacks jitter at up to 1.7 m/s and a
         * sliding cube diverge; on friction rows only, a cube on a 35
         * degree slope slid 0.47, 2.08, 7.51 and 3.28 m at 1, 2, 4 and 8
         * iterations against 3.22 m, worse than this monotone
         * convergence (docs/rigid.md). */
        const float k_h = row->k;
        for (int a = 0; a < 6; a++) {
            f[a] -= j[a] * force;
            for (int b = 0; b < 6; b++) {
                hm[a][b] += k_h * j[a] * j[b];
            }
        }
        /* Diagonal geometric stiffness bound (paper Eq. 17). */
        const float g = sl_abs(force) * sl_vec3_length(rr);
        hm[3][3] += g;
        hm[4][4] += g;
        hm[5][5] += g;
    }
    for (uint32_t ref = st->body_contact_offset[i];
         ref < st->body_contact_offset[i + 1u]; ref++) {
        rc_alm_contact_block(s, &st->contacts[st->body_contact[ref]], i, h, f,
                             hm);
    }
    float step[6];
    if (!rc_solve6(hm, f, step)) {
        return;
    }
    s->position[i] =
        sl_vec3_add(s->position[i], sl_vec3_make(step[0], step[1], step[2]));
    const sl_quat q = s->orientation[i];
    const sl_quat w4 =
        sl_quat_mul(sl_quat_make(step[3], step[4], step[5], 0.0f), q);
    s->orientation[i] =
        sl_quat_normalize(sl_quat_make(q.x + 0.5f * w4.x, q.y + 0.5f * w4.y,
                                       q.z + 0.5f * w4.z, q.w + 0.5f * w4.w));
}

static float rc_row_mass(const sl_body_set *s, uint32_t a, uint32_t b)
{
    const float ma = (rc_inverse_mass(s, a) > 0.0f)
                         ? 1.0f / rc_inverse_mass(s, a)
                         : INFINITY;
    const float mb = (rc_inverse_mass(s, b) > 0.0f)
                         ? 1.0f / rc_inverse_mass(s, b)
                         : INFINITY;
    return sl_min(ma, mb);
}

static void rc_add_row(rc_avbd_state *st, const sl_body_set *s, rc_row row,
                       const rc_avbd_params *prm, float h)
{
    if (prm->alm && !prm->scaled) {
        /* Fixed penalty: the row's inverse-Delassus support / h^2 (set by
         * the caller in k_start); multiplier retained by gamma. */
        row.k = row.k_start;
        row.beta = 0.0f;
        row.lambda =
            (row.warm_lambda != NULL) ? prm->gamma * *row.warm_lambda : 0.0f;
        st->rows[st->row_count++] = row;
        return;
    }
    if (prm->scaled) {
        const float base = rc_row_mass(s, row.a, row.b) / (h * h);
        row.k_start = base;
        row.beta = prm->beta * base;
    } else {
        row.k_start = RC_PENALTY_MIN;
        row.beta = prm->beta;
    }
    /* Warm start (paper Eq. 19). */
    if (row.warm_k != NULL && *row.warm_k > 0.0f) {
        row.k = sl_max(prm->gamma * *row.warm_k, row.k_start);
        row.lambda = prm->alpha * prm->gamma * *row.warm_lambda;
    } else {
        row.k = row.k_start;
        row.lambda = 0.0f;
    }
    st->rows[st->row_count++] = row;
}

/* Velocity-level Coulomb friction after the position solve, as silk's
 * cloth does (docs/solver.md): per normal row, the tangential relative
 * velocity at the contact is reduced by at most mu |F_n| h W, with W the
 * effective inverse mass along it, one Gauss-Seidel pass. */
static void rc_avbd_velocity_friction(rc_world *w, rc_avbd_state *st, float h,
                                      const rc_avbd_params *prm)
{
    sl_body_set *s = &w->set;
    for (uint32_t r = 0; r < st->row_count; r++) {
        const rc_row *row = &st->rows[r];
        if (row->kind != RC_ROW_NORMAL) {
            continue;
        }
        float c;
        const float fn = rc_row_force(s, st->rows, row, prm->alpha, &c);
        if (!(fn < 0.0f)) {
            continue;
        }
        const sl_vec3 pa = rc_anchor(s, row->a, row->local_a);
        const sl_vec3 ra = sl_vec3_sub(pa, s->position[row->a]);
        const sl_vec3 rb = (row->b >= SL_RIGID_STATIC_BASE)
                               ? sl_vec3_zero()
                               : sl_vec3_sub(rc_anchor(s, row->b, row->local_b),
                                             s->position[row->b]);
        const sl_vec3 dv = sl_vec3_sub(rc_point_velocity(s, row->b, rb),
                                       rc_point_velocity(s, row->a, ra));
        const sl_vec3 n = row->d;
        const sl_vec3 vt = sl_vec3_mul_add(dv, n, -sl_vec3_dot(dv, n));
        const float speed = sl_vec3_length(vt);
        if (!(speed > 0.0f)) {
            continue;
        }
        const sl_vec3 t = sl_vec3_scale(vt, 1.0f / speed);
        /* Effective inverse mass along t. */
        float wsum = rc_inverse_mass(s, row->a) + rc_inverse_mass(s, row->b);
        sl_mat3 ia = sl_mat3_zero();
        sl_mat3 ib = sl_mat3_zero();
        if (row->a < SL_RIGID_STATIC_BASE && s->inverse_mass[row->a] > 0.0f) {
            ia = rc_inverse_inertia_world(s, row->a);
            const sl_vec3 x = sl_vec3_cross(ra, t);
            wsum += sl_vec3_dot(x, sl_mat3_mul_vec3(ia, x));
        }
        if (row->b < SL_RIGID_STATIC_BASE && s->inverse_mass[row->b] > 0.0f) {
            ib = rc_inverse_inertia_world(s, row->b);
            const sl_vec3 x = sl_vec3_cross(rb, t);
            wsum += sl_vec3_dot(x, sl_mat3_mul_vec3(ib, x));
        }
        if (!(wsum > 0.0f)) {
            continue;
        }
        const float stop = speed / wsum;           /* impulse that stops it */
        const float limit = RC_FRICTION * -fn * h; /* Coulomb bound */
        const float j = sl_min(stop, limit);
        const sl_vec3 impulse = sl_vec3_scale(t, -j);
        if (row->a < SL_RIGID_STATIC_BASE && s->inverse_mass[row->a] > 0.0f) {
            s->velocity[row->a] = sl_vec3_mul_add(s->velocity[row->a], impulse,
                                                  -s->inverse_mass[row->a]);
            s->angular_velocity[row->a] =
                sl_vec3_sub(s->angular_velocity[row->a],
                            sl_mat3_mul_vec3(ia, sl_vec3_cross(ra, impulse)));
        }
        if (row->b < SL_RIGID_STATIC_BASE && s->inverse_mass[row->b] > 0.0f) {
            s->velocity[row->b] = sl_vec3_mul_add(s->velocity[row->b], impulse,
                                                  s->inverse_mass[row->b]);
            s->angular_velocity[row->b] =
                sl_vec3_add(s->angular_velocity[row->b],
                            sl_mat3_mul_vec3(ib, sl_vec3_cross(rb, impulse)));
        }
    }
}

/* Joint rows (three world axes per ball joint). */
static void rc_add_joint_rows(rc_avbd_state *st, rc_world *w,
                              const rc_avbd_params *prm, float h)
{
    sl_body_set *s = &w->set;
    for (uint32_t k = 0; k < w->joint_count; k++) {
        rc_joint *j = &w->joints[k];
        const sl_vec3 diff = sl_vec3_sub(rc_anchor(s, j->b, j->local_b),
                                         rc_anchor(s, j->a, j->local_a));
        const sl_vec3 axes[3] = { sl_vec3_make(1, 0, 0), sl_vec3_make(0, 1, 0),
                                  sl_vec3_make(0, 0, 1) };
        for (uint32_t a = 0; a < 3u; a++) {
            rc_row row;
            memset(&row, 0, sizeof(row));
            row.a = j->a;
            row.b = j->b;
            row.local_a = j->local_a;
            row.local_b = j->local_b;
            row.kind = RC_ROW_JOINT;
            row.d = axes[a];
            row.c0 = sl_vec3_dot(diff, axes[a]);
            row.warm_lambda = &j->lambda[a];
            row.warm_k = &j->k[a];
            if (prm->alm) {
                const sl_vec3 pa = rc_anchor(s, j->a, j->local_a);
                const sl_vec3 pb = rc_anchor(s, j->b, j->local_b);
                const float wj =
                    rc_delassus(s, j->a, rc_lever(s, j->a, pa), axes[a]) +
                    rc_delassus(s, j->b, rc_lever(s, j->b, pb), axes[a]);
                row.k_start = (wj > 0.0f) ? 1.0f / (wj * h * h) : 0.0f;
            }
            rc_add_row(st, s, row, prm, h);
        }
    }
}

/* Body -> row references (CSR). */
static void rc_index_rows(rc_avbd_state *st, const sl_body_set *s)
{
    memset(st->body_row_offset, 0, sizeof(uint32_t) * (s->count + 1u));
    for (uint32_t r = 0; r < st->row_count; r++) {
        const rc_row *row = &st->rows[r];
        if (row->a < s->count && s->inverse_mass[row->a] > 0.0f) {
            st->body_row_offset[row->a + 1u]++;
        }
        if (row->b < s->count && s->inverse_mass[row->b] > 0.0f) {
            st->body_row_offset[row->b + 1u]++;
        }
    }
    for (uint32_t i = 0; i < s->count; i++) {
        st->body_row_offset[i + 1u] += st->body_row_offset[i];
    }
    memcpy(st->body_row_fill, st->body_row_offset,
           sizeof(uint32_t) * (s->count + 1u));
    for (uint32_t r = 0; r < st->row_count; r++) {
        const rc_row *row = &st->rows[r];
        if (row->a < s->count && s->inverse_mass[row->a] > 0.0f) {
            st->body_row[st->body_row_fill[row->a]++] = r;
        }
        if (row->b < s->count && s->inverse_mass[row->b] > 0.0f) {
            st->body_row[st->body_row_fill[row->b]++] = r;
        }
    }
}

/* Joint pass for Soft Step (soft+joints): with every body's inertia
 * anchored at its Soft Step position x_p, minimizes
 * 1/2 |x - x_p|^2_M / h^2 plus the joints' augmented Lagrangian (scaled
 * AVBD rows, alpha = 0 so each substep removes the whole error), by
 * block Gauss-Seidel over jointed bodies; the correction is added to the
 * velocities as (x - x_p) / h. Contacts are not in this pass: they are
 * solved by Soft Step before it and relaxed after. */
static void rc_joint_pass(rc_world *w, rc_avbd_state *st, float h,
                          uint32_t iterations)
{
    sl_body_set *s = &w->set;
    rc_avbd_params prm = rc_avbd_params_of(RC_AVBD_SCALED);
    prm.alpha = 0.0f;
    st->row_count = 0;
    st->contact_count = 0;
    rc_add_joint_rows(st, w, &prm, h);
    rc_index_rows(st, s);
    memset(st->body_contact_offset, 0, sizeof(uint32_t) * (s->count + 1u));
    for (uint32_t i = 0; i < s->count; i++) {
        st->x0[i] = s->position[i];
        st->q0[i] = s->orientation[i];
        st->x_inertial[i] = s->position[i];
        st->q_inertial[i] = s->orientation[i];
    }
    for (uint32_t it = 0; it < iterations; it++) {
        for (uint32_t i = 0; i < s->count; i++) {
            if (s->inverse_mass[i] > 0.0f &&
                st->body_row_offset[i + 1u] > st->body_row_offset[i]) {
                rc_avbd_body(w, st, i, h, &prm);
            }
        }
        for (uint32_t r = 0; r < st->row_count; r++) {
            rc_row *row = &st->rows[r];
            float c;
            const float force = rc_row_force(s, st->rows, row, prm.alpha, &c);
            row->k = sl_min(RC_PENALTY_MAX, row->k + row->beta * sl_abs(c));
            row->lambda = force;
        }
    }
    for (uint32_t r = 0; r < st->row_count; r++) {
        *st->rows[r].warm_lambda = st->rows[r].lambda;
        *st->rows[r].warm_k = st->rows[r].k;
    }
    for (uint32_t i = 0; i < s->count; i++) {
        if (s->inverse_mass[i] == 0.0f ||
            st->body_row_offset[i + 1u] == st->body_row_offset[i]) {
            continue;
        }
        s->velocity[i] = sl_vec3_mul_add(
            s->velocity[i], sl_vec3_sub(s->position[i], st->x0[i]), 1.0f / h);
        sl_quat dq =
            sl_quat_mul(s->orientation[i], sl_quat_conjugate(st->q0[i]));
        if (dq.w < 0.0f) {
            dq = sl_quat_make(-dq.x, -dq.y, -dq.z, -dq.w);
        }
        s->angular_velocity[i] = sl_vec3_add(
            s->angular_velocity[i],
            sl_vec3_make(2.0f * dq.x / h, 2.0f * dq.y / h, 2.0f * dq.z / h));
    }
}

static void rc_avbd_step(rc_world *w, uint32_t iterations, float h,
                         bool reverse, const rc_avbd_params *prm,
                         rc_avbd_state *st)
{
    sl_body_set *s = &w->set;
    /* Rows: contacts (normal + two friction) and joints (three axes);
     * under compliant ALM, contacts are vector contacts instead. */
    st->row_count = 0;
    st->contact_count = 0;
    for (uint32_t m = 0; m < s->manifold_count; m++) {
        sl_manifold *mf = &s->manifolds[m];
        sl_vec3 t1;
        sl_vec3 t2;
        rc_tangents(mf->normal, &t1, &t2);
        for (uint32_t q = 0; q < mf->point_count; q++) {
            sl_contact_point *cp = &mf->points[q];
            if (prm->alm) {
                rc_alm_contact *c = &st->contacts[st->contact_count++];
                c->a = mf->body_a;
                c->b = mf->body_b;
                c->local_a = cp->local_a;
                c->local_b = cp->local_b;
                c->n = mf->normal;
                c->pa0 = rc_anchor(s, c->a, c->local_a);
                c->pb0 = rc_anchor(s, c->b, c->local_b);
                const sl_vec3 ra = rc_lever(s, c->a, c->pa0);
                const sl_vec3 rb = rc_lever(s, c->b, c->pb0);
                const float wn = rc_delassus(s, c->a, ra, c->n) +
                                 rc_delassus(s, c->b, rb, c->n);
                const float wt = 0.5f * (rc_delassus(s, c->a, ra, t1) +
                                         rc_delassus(s, c->b, rb, t1) +
                                         rc_delassus(s, c->a, ra, t2) +
                                         rc_delassus(s, c->b, rb, t2));
                c->rho_n = (wn > 0.0f) ? 1.0f / (wn * h * h) : 0.0f;
                c->beta = prm->scaled ? prm->beta * c->rho_n : 0.0f;
                c->alpha = prm->alpha;
                c->depth0 = sl_max(
                    -sl_vec3_dot(c->n, sl_vec3_sub(c->pb0, c->pa0)), 0.0f);
                c->rho_t = (wt > 0.0f) ? 1.0f / (wt * h * h) : 0.0f;
                c->warm = cp->warm;
                const float retain =
                    prm->scaled ? prm->alpha * prm->gamma : prm->gamma;
                c->lambda = sl_vec3_scale(
                    sl_vec3_make(cp->warm[0], cp->warm[1], cp->warm[2]),
                    retain);
                if (prm->scaled) {
                    /* Ramped penalties warm-start like AVBD's k. */
                    c->rho_n = sl_max(prm->gamma * cp->warm[3], c->rho_n);
                }
                continue;
            }
            rc_row row;
            memset(&row, 0, sizeof(row));
            row.a = mf->body_a;
            row.b = mf->body_b;
            row.local_a = cp->local_a;
            row.local_b = cp->local_b;
            const sl_vec3 diff = sl_vec3_sub(rc_anchor(s, row.b, row.local_b),
                                             rc_anchor(s, row.a, row.local_a));
            row.kind = RC_ROW_NORMAL;
            row.d = mf->normal;
            row.c0 = sl_vec3_dot(diff, mf->normal);
            row.warm_lambda = &cp->warm[0];
            row.warm_k = &cp->warm[3];
            const uint32_t normal_index = st->row_count;
            rc_add_row(st, s, row, prm, h);
            const sl_vec3 dirs[2] = { t1, t2 };
            for (uint32_t t = 0; t < 2u && !prm->velocity_friction; t++) {
                rc_row fr = row;
                fr.kind = RC_ROW_FRICTION;
                fr.d = dirs[t];
                fr.c0 = 0.0f;
                fr.start = diff;
                fr.normal = normal_index;
                fr.warm_lambda = &cp->warm[1u + t];
                fr.warm_k = &cp->warm[4u + t];
                rc_add_row(st, s, fr, prm, h);
            }
        }
    }
    rc_add_joint_rows(st, w, prm, h);
    rc_index_rows(st, s);
    memset(st->body_contact_offset, 0, sizeof(uint32_t) * (s->count + 1u));
    for (uint32_t k = 0; k < st->contact_count; k++) {
        const rc_alm_contact *c = &st->contacts[k];
        if (c->a < s->count && s->inverse_mass[c->a] > 0.0f) {
            st->body_contact_offset[c->a + 1u]++;
        }
        if (c->b < s->count && s->inverse_mass[c->b] > 0.0f) {
            st->body_contact_offset[c->b + 1u]++;
        }
    }
    for (uint32_t i = 0; i < s->count; i++) {
        st->body_contact_offset[i + 1u] += st->body_contact_offset[i];
    }
    memcpy(st->body_row_fill, st->body_contact_offset,
           sizeof(uint32_t) * (s->count + 1u));
    for (uint32_t k = 0; k < st->contact_count; k++) {
        const rc_alm_contact *c = &st->contacts[k];
        if (c->a < s->count && s->inverse_mass[c->a] > 0.0f) {
            st->body_contact[st->body_row_fill[c->a]++] = k;
        }
        if (c->b < s->count && s->inverse_mass[c->b] > 0.0f) {
            st->body_contact[st->body_row_fill[c->b]++] = k;
        }
    }
    /* Inertial targets and the adaptive initial guess (VBD; avbd demo). */
    for (uint32_t i = 0; i < s->count; i++) {
        st->x0[i] = s->position[i];
        st->q0[i] = s->orientation[i];
        if (s->inverse_mass[i] == 0.0f) {
            st->x_inertial[i] = s->position[i];
            st->q_inertial[i] = s->orientation[i];
            continue;
        }
        const sl_vec3 v = s->velocity[i];
        st->x_inertial[i] =
            sl_vec3_add(sl_vec3_mul_add(s->position[i], v, h),
                        sl_vec3_make(0.0f, RC_GRAVITY * h * h, 0.0f));
        const sl_vec3 omega = s->angular_velocity[i];
        const sl_quat q = s->orientation[i];
        const sl_quat dq =
            sl_quat_mul(sl_quat_make(omega.x, omega.y, omega.z, 0.0f), q);
        st->q_inertial[i] = sl_quat_normalize(
            sl_quat_make(q.x + 0.5f * h * dq.x, q.y + 0.5f * h * dq.y,
                         q.z + 0.5f * h * dq.z, q.w + 0.5f * h * dq.w));
        const float accel = (v.y - w->velocity_previous[i].y) / h;
        const float weight = sl_clamp(accel / RC_GRAVITY, 0.0f, 1.0f);
        s->position[i] =
            sl_vec3_add(sl_vec3_mul_add(s->position[i], v, h),
                        sl_vec3_make(0.0f, weight * RC_GRAVITY * h * h, 0.0f));
        s->orientation[i] = st->q_inertial[i];
    }
    for (uint32_t it = 0; it < iterations; it++) {
        for (uint32_t k = 0; k < s->count; k++) {
            const uint32_t i = reverse ? s->count - 1u - k : k;
            if (s->inverse_mass[i] > 0.0f) {
                rc_avbd_body(w, st, i, h, prm);
            }
        }
        /* Dual update (paper Eqs. 11-13). */
        for (uint32_t r = 0; r < st->row_count; r++) {
            rc_row *row = &st->rows[r];
            float c;
            const float force = rc_row_force(s, st->rows, row, prm->alpha, &c);
            const float unclamped = row->k * c + row->lambda;
            if (force == unclamped) {
                row->k = sl_min(RC_PENALTY_MAX, row->k + row->beta * sl_abs(c));
            }
            row->lambda = force;
        }
        for (uint32_t k = 0; k < st->contact_count; k++) {
            rc_alm_contact_dual(s, &st->contacts[k]);
        }
    }
    for (uint32_t r = 0; r < st->row_count; r++) {
        *st->rows[r].warm_lambda = st->rows[r].lambda;
        *st->rows[r].warm_k = st->rows[r].k;
    }
    for (uint32_t k = 0; k < st->contact_count; k++) {
        rc_alm_contact *c = &st->contacts[k];
        c->warm[0] = c->lambda.x;
        c->warm[1] = c->lambda.y;
        c->warm[2] = c->lambda.z;
        c->warm[3] = c->rho_n;
    }
    for (uint32_t i = 0; i < s->count; i++) {
        if (s->inverse_mass[i] == 0.0f) {
            continue;
        }
        w->velocity_previous[i] = s->velocity[i];
        s->velocity[i] =
            sl_vec3_scale(sl_vec3_sub(s->position[i], st->x0[i]), 1.0f / h);
        sl_quat dq =
            sl_quat_mul(s->orientation[i], sl_quat_conjugate(st->q0[i]));
        if (dq.w < 0.0f) {
            dq = sl_quat_make(-dq.x, -dq.y, -dq.z, -dq.w);
        }
        s->angular_velocity[i] =
            sl_vec3_make(2.0f * dq.x / h, 2.0f * dq.y / h, 2.0f * dq.z / h);
    }
    if (prm->velocity_friction) {
        rc_avbd_velocity_friction(w, st, h, prm);
    }
}

/* ---------------------------------------------------------------------- */
/* Scenes and metrics                                                      */

typedef enum rc_solver {
    RC_SOFT = 0,
    RC_AVBD_PAPER_S = 1,
    RC_AVBD_DEMO_S = 2,
    RC_AVBD_SCALED_S = 3,
    RC_AVBD_SCALED_VF_S = 4,
    RC_AVBD_ALM_S = 5,
    RC_AVBD_ALM_SUB_S = 6,
    RC_AVBD_ALM_STAB_S = 7,
    RC_AVBD_ALM_STAB_SUB_S = 8,
    RC_AVBD_HYBRID_S = 9,
    RC_SOFT_JOINTS_S = 10,
    RC_SOFT_JOINTS_FIRST_S = 11,
    RC_SOLVER_COUNT = 12
} rc_solver;

static const char *const k_solver_names[RC_SOLVER_COUNT] = {
    "soft-step",    "avbd-paper", "avbd-demo",   "avbd-scaled",
    "avbd-vfric",   "avbd-alm",   "alm-substep", "alm-stab",
    "alm-stab-sub", "hybrid",     "soft+joints", "joints-first"
};

typedef struct rc_metrics {
    float penetration; /* deepest overlap at any detection, last second */
    float speed;       /* max body speed, last second (rest jitter) */
    float drift;       /* max horizontal displacement of any body */
    float drop;        /* largest height loss of any body (collapse) */
    float stretch;     /* chain: max relative elongation */
    float slide;       /* slope: displacement along the slope */
    float joint_gap;   /* largest anchor separation of any joint, m */
    double ms;         /* solver + collision per step */
} rc_metrics;

typedef struct rc_scene {
    const char *name;
    const char *about;
    bool (*build)(rc_world *w, float param);
    float param;
    uint32_t steps;
} rc_scene;

static bool rc_ground(rc_world *w, sl_vec3 origin)
{
    w->planes[0].point = origin;
    w->planes[0].normal = sl_vec3_make(0.0f, 1.0f, 0.0f);
    w->plane_count = 1;
    return true;
}

static sl_vec3 rc_origin(float far)
{
    return sl_vec3_make(far, 0.0f, far);
}

static bool rc_build_stack(rc_world *w, float density, uint32_t count,
                           float far, bool reverse_order)
{
    if (!rc_world_init(w, count)) {
        return false;
    }
    rc_ground(w, rc_origin(far));
    const sl_vec3 h = sl_vec3_make(0.5f, 0.5f, 0.5f);
    for (uint32_t k = 0; k < count; k++) {
        const uint32_t level = reverse_order ? count - 1u - k : k;
        uint32_t body;
        if (!sl_body_add_box(
                &w->set,
                sl_vec3_add(rc_origin(far),
                            sl_vec3_make(0.0f, 0.5f + (float)level, 0.0f)),
                sl_quat_identity(), h, density, &body)) {
            return false;
        }
    }
    return true;
}

static bool rc_scene_stack(rc_world *w, float param)
{
    return rc_build_stack(w, 1000.0f, (uint32_t)param, 0.0f, false);
}

static bool rc_scene_density(rc_world *w, float param)
{
    return rc_build_stack(w, param, 10u, 0.0f, false);
}

static bool rc_scene_far(rc_world *w, float param)
{
    return rc_build_stack(w, 1000.0f, 10u, param, false);
}

static bool rc_scene_reversed(rc_world *w, float param)
{
    (void)param;
    return rc_build_stack(w, 1000.0f, 10u, 0.0f, true);
}

static bool rc_scene_pyramid(rc_world *w, float param)
{
    const uint32_t base = (uint32_t)param;
    if (!rc_world_init(w, base * (base + 1u) / 2u)) {
        return false;
    }
    rc_ground(w, sl_vec3_zero());
    const sl_vec3 h = sl_vec3_make(0.5f, 0.5f, 0.5f);
    for (uint32_t row = 0; row < base; row++) {
        for (uint32_t k = 0; k < base - row; k++) {
            const float x = (float)k * 1.05f + 0.525f * (float)row;
            uint32_t body;
            if (!sl_body_add_box(&w->set,
                                 sl_vec3_make(x, 0.5f + (float)row, 0.0f),
                                 sl_quat_identity(), h, 1000.0f, &body)) {
                return false;
            }
        }
    }
    return true;
}

/* A 50-link chain hung horizontally from a static anchor, with a heavy box
 * at its end: link mass 1/ratio of the end box. */
static bool rc_build_chain(rc_world *w, float ratio, float height, bool drape)
{
    const uint32_t links = 50u;
    if (!rc_world_init(w, links + 3u)) {
        return false;
    }
    w->plane_count = 0;
    const sl_vec3 hl = sl_vec3_make(0.25f, 0.05f, 0.05f);
    uint32_t anchor;
    if (!sl_body_add_box(&w->set, sl_vec3_make(0.0f, height, 0.0f),
                         sl_quat_identity(), sl_vec3_make(0.1f, 0.1f, 0.1f),
                         0.0f, &anchor)) {
        return false;
    }
    uint32_t prev = anchor;
    sl_vec3 prev_anchor = sl_vec3_zero();
    const float link_density = 100.0f;
    for (uint32_t k = 0; k < links; k++) {
        uint32_t body;
        const float x = 0.25f + 0.5f * (float)k;
        if (!sl_body_add_box(&w->set, sl_vec3_make(x, height, 0.0f),
                             sl_quat_identity(), hl, link_density, &body)) {
            return false;
        }
        rc_joint *j = &w->joints[w->joint_count++];
        memset(j, 0, sizeof(*j));
        j->a = prev;
        j->b = body;
        j->local_a = prev_anchor;
        j->local_b = sl_vec3_make(-0.25f, 0.0f, 0.0f);
        prev = body;
        prev_anchor = sl_vec3_make(0.25f, 0.0f, 0.0f);
    }
    /* End box: mass ratio times one link. */
    const float link_mass = link_density * 8.0f * hl.x * hl.y * hl.z;
    const sl_vec3 he = sl_vec3_make(0.25f, 0.25f, 0.25f);
    const float end_density = ratio * link_mass / (8.0f * he.x * he.y * he.z);
    uint32_t end;
    if (!sl_body_add_box(
            &w->set, sl_vec3_make(0.5f * (float)links + 0.25f, height, 0.0f),
            sl_quat_identity(), he, end_density, &end)) {
        return false;
    }
    rc_joint *j = &w->joints[w->joint_count++];
    memset(j, 0, sizeof(*j));
    j->a = prev;
    j->b = end;
    j->local_a = prev_anchor;
    j->local_b = sl_vec3_make(-0.25f, 0.0f, 0.0f);
    if (drape) {
        /* A static block under the chain's first third and the ground:
         * the chain falls across the block, so contacts push links while
         * joints hold them. */
        uint32_t block;
        if (!sl_body_add_box(&w->set, sl_vec3_make(8.0f, 2.0f, 0.0f),
                             sl_quat_identity(), sl_vec3_make(1.5f, 0.5f, 1.5f),
                             0.0f, &block)) {
            return false;
        }
        rc_ground(w, sl_vec3_zero());
    }
    return true;
}

/* A 50-link chain hung horizontally at 30 m from a static anchor. */
static bool rc_scene_chain(rc_world *w, float ratio)
{
    return rc_build_chain(w, ratio, 30.0f, false);
}

/* The chain anchored at 4 m, falling across a block onto the ground. */
static bool rc_scene_drape(rc_world *w, float ratio)
{
    return rc_build_chain(w, ratio, 4.0f, true);
}

/* A 1 m cube on a plane tilted by param degrees about z, friction 0.5. */
static bool rc_scene_slope(rc_world *w, float degrees)
{
    if (!rc_world_init(w, 1u)) {
        return false;
    }
    const float theta = degrees * SL_PI / 180.0f;
    w->planes[0].point = sl_vec3_zero();
    w->planes[0].normal = sl_vec3_make(-sinf(theta), cosf(theta), 0.0f);
    w->plane_count = 1;
    uint32_t body;
    const sl_quat q = sl_quat_from_axis_angle(sl_vec3_make(0, 0, 1), theta);
    return sl_body_add_box(&w->set, sl_vec3_scale(w->planes[0].normal, 0.5f), q,
                           sl_vec3_make(0.5f, 0.5f, 0.5f), 1000.0f, &body);
}

static const rc_scene k_scenes[] = {
    { "stack-10", "10 cubes stacked", rc_scene_stack, 10.0f, 300u },
    { "stack-30", "30 cubes stacked", rc_scene_stack, 30.0f, 300u },
    { "pyramid-20", "pyramid, 20-box base (210 boxes)", rc_scene_pyramid, 20.0f,
      300u },
    { "density-1", "10-stack at 1 kg/m^3", rc_scene_density, 1.0f, 300u },
    { "density-1e5", "10-stack at 1e5 kg/m^3", rc_scene_density, 1.0e5f, 300u },
    { "far-5km", "10-stack at (5 km, 0, 5 km)", rc_scene_far, 5000.0f, 300u },
    { "reversed", "10-stack, bodies created top-down", rc_scene_reversed, 0.0f,
      300u },
    { "chain-1", "50-link chain, end mass 1 link", rc_scene_chain, 1.0f, 300u },
    { "chain-1e3", "50-link chain, end mass 1e3 links", rc_scene_chain, 1.0e3f,
      300u },
    { "chain-5e4", "50-link chain, end mass 5e4 links", rc_scene_chain, 5.0e4f,
      300u },
    { "drape-1e2", "chain over a block onto ground, end mass 1e2 links",
      rc_scene_drape, 1.0e2f, 300u },
    { "drape-1e3", "chain over a block onto ground, end mass 1e3 links",
      rc_scene_drape, 1.0e3f, 300u },
    { "slope-20", "cube on a 20 deg slope, mu 0.5 (sticks)", rc_scene_slope,
      20.0f, 120u },
    { "slope-35", "cube on a 35 deg slope, mu 0.5 (slides 3.22 m in 2 s)",
      rc_scene_slope, 35.0f, 120u },
};

/* Removes contacts between bodies joined by a joint, as engines do by
 * default: chain links touch where they are joined, and their contacts
 * would fight the joints. */
static void rc_drop_jointed(rc_world *w)
{
    sl_body_set *s = &w->set;
    uint32_t kept = 0;
    for (uint32_t m = 0; m < s->manifold_count; m++) {
        const sl_manifold *mf = &s->manifolds[m];
        bool jointed = false;
        for (uint32_t k = 0; k < w->joint_count && !jointed; k++) {
            const rc_joint *j = &w->joints[k];
            jointed = (mf->body_a == j->a && mf->body_b == j->b) ||
                      (mf->body_a == j->b && mf->body_b == j->a);
        }
        if (!jointed) {
            s->manifolds[kept++] = *mf;
        }
    }
    s->manifold_count = kept;
}

static rc_metrics rc_run(const rc_scene *scene, rc_solver solver,
                         uint32_t budget)
{
    rc_metrics m;
    memset(&m, 0, sizeof(m));
    rc_world w;
    if (!scene->build(&w, scene->param)) {
        m.ms = -1.0;
        rc_world_destroy(&w);
        return m;
    }
    sl_body_set *s = &w.set;
    rc_soft_state soft;
    rc_avbd_state avbd;
    memset(&soft, 0, sizeof(soft));
    memset(&avbd, 0, sizeof(avbd));
    const uint32_t points_max =
        SL_MANIFOLD_POINT_COUNT_MAX * s->manifold_capacity;
    soft.points = calloc(points_max, sizeof(rc_point));
    soft.inverse_inertia = calloc(s->capacity, sizeof(sl_mat3));
    soft.saved_position = calloc(s->capacity, sizeof(sl_vec3));
    soft.saved_orientation = calloc(s->capacity, sizeof(sl_quat));
    const uint32_t rows_max = 3u * points_max + 3u * RC_JOINT_MAX;
    avbd.rows = calloc(rows_max, sizeof(rc_row));
    avbd.contacts = calloc(points_max, sizeof(rc_alm_contact));
    avbd.body_contact_offset = calloc(s->capacity + 1u, sizeof(uint32_t));
    avbd.body_contact = calloc(2u * points_max, sizeof(uint32_t));
    avbd.body_row_offset = calloc(s->capacity + 1u, sizeof(uint32_t));
    avbd.body_row_fill = calloc(s->capacity + 1u, sizeof(uint32_t));
    avbd.body_row = calloc(2u * rows_max, sizeof(uint32_t));
    avbd.x0 = calloc(s->capacity, sizeof(sl_vec3));
    avbd.q0 = calloc(s->capacity, sizeof(sl_quat));
    avbd.x_inertial = calloc(s->capacity, sizeof(sl_vec3));
    avbd.q_inertial = calloc(s->capacity, sizeof(sl_quat));
    sl_vec3 *start = calloc(s->capacity, sizeof(sl_vec3));
    for (uint32_t i = 0; i < s->count; i++) {
        start[i] = s->position[i];
    }
    const rc_avbd_params prm = rc_avbd_params_of(
        (solver == RC_AVBD_DEMO_S)           ? RC_AVBD_DEMO
        : (solver == RC_AVBD_SCALED_S)       ? RC_AVBD_SCALED
        : (solver == RC_AVBD_SCALED_VF_S)    ? RC_AVBD_SCALED_VF
        : (solver == RC_AVBD_ALM_S)          ? RC_AVBD_ALM
        : (solver == RC_AVBD_ALM_SUB_S)      ? RC_AVBD_ALM_SUB
        : (solver == RC_AVBD_ALM_STAB_S)     ? RC_AVBD_ALM_STAB
        : (solver == RC_AVBD_ALM_STAB_SUB_S) ? RC_AVBD_ALM_STAB_SUB
        : (solver == RC_AVBD_HYBRID_S)       ? RC_AVBD_HYBRID
                                             : RC_AVBD_PAPER);
    const bool reverse = false;
    double total = 0.0;
    float rest_length = 0.0f;
    if (w.joint_count > 0u) {
        rest_length = 0.5f * (float)(w.joint_count - 1u) + 0.25f;
    }
    for (uint32_t step = 0; step < scene->steps; step++) {
        const double t0 = rc_seconds();
        sl_rigid_collide(s, w.planes, w.plane_count, RC_MARGIN);
        rc_drop_jointed(&w);
        if (solver == RC_SOFT || solver == RC_SOFT_JOINTS_S ||
            solver == RC_SOFT_JOINTS_FIRST_S) {
            rc_soft_step(&w, budget, reverse, &soft,
                         (solver == RC_SOFT) ? NULL : &avbd,
                         solver == RC_SOFT_JOINTS_FIRST_S);
        } else {
            /* Substepped variants spend the budget on substeps of one
             * iteration each, reusing the step's contacts (as Soft Step
             * does); the others on iterations of one full step. */
            const uint32_t substeps = prm.substepped ? budget : 1u;
            const uint32_t iterations = prm.substepped ? 1u : budget;
            for (uint32_t sub = 0; sub < substeps; sub++) {
                rc_avbd_step(&w, iterations, RC_DT / (float)substeps, reverse,
                             &prm, &avbd);
            }
        }
        total += rc_seconds() - t0;
        const bool last_second = step + 60u >= scene->steps;
        for (uint32_t k = 0; k < s->manifold_count && last_second; k++) {
            for (uint32_t q = 0; q < s->manifolds[k].point_count; q++) {
                m.penetration = sl_max(m.penetration,
                                       -s->manifolds[k].points[q].separation);
            }
        }
        for (uint32_t i = 0; i < s->count; i++) {
            if (s->inverse_mass[i] == 0.0f) {
                continue;
            }
            const sl_vec3 p = s->position[i];
            if (!sl_vec3_is_finite(p)) {
                m.drop = INFINITY;
                continue;
            }
            const sl_vec3 d = sl_vec3_sub(p, start[i]);
            if (last_second) {
                m.speed = sl_max(m.speed, sl_vec3_length(s->velocity[i]));
            }
            m.drift = sl_max(m.drift, sqrtf(d.x * d.x + d.z * d.z));
            m.drop = sl_max(m.drop, -d.y);
        }
        for (uint32_t k = 0; k < w.joint_count; k++) {
            const rc_joint *j = &w.joints[k];
            const float gap =
                sl_vec3_length(sl_vec3_sub(rc_anchor(s, j->b, j->local_b),
                                           rc_anchor(s, j->a, j->local_a)));
            m.joint_gap = sl_max(m.joint_gap, gap);
        }
        if (w.joint_count > 0u) {
            const sl_vec3 end = s->position[w.joints[w.joint_count - 1u].b];
            const float length =
                sl_vec3_length(sl_vec3_sub(end, s->position[w.joints[0].a]));
            m.stretch = sl_max(m.stretch, (length - rest_length) / rest_length);
        }
    }
    if (w.plane_count == 1u && s->count == 1u) {
        const sl_vec3 n = w.planes[0].normal;
        const sl_vec3 down = sl_vec3_make(-n.y, n.x, 0.0f); /* along slope */
        m.slide = sl_vec3_dot(sl_vec3_sub(s->position[0], start[0]), down);
    }
    m.ms = 1000.0 * total / (double)scene->steps;
    free(start);
    free(soft.points);
    free(soft.inverse_inertia);
    free(soft.saved_position);
    free(soft.saved_orientation);
    free(avbd.rows);
    free(avbd.contacts);
    free(avbd.body_contact_offset);
    free(avbd.body_contact);
    free(avbd.body_row_offset);
    free(avbd.body_row_fill);
    free(avbd.body_row);
    free(avbd.x0);
    free(avbd.q0);
    free(avbd.x_inertial);
    free(avbd.q_inertial);
    rc_world_destroy(&w);
    return m;
}

int main(int argc, char **argv)
{
    bool quick = false;
    const char *only = NULL;
    for (int a = 1; a < argc; a++) {
        if (strcmp(argv[a], "--quick") == 0) {
            quick = true;
        } else if (a + 1 < argc && strcmp(argv[a], "--scene") == 0) {
            only = argv[++a];
        }
    }
    const uint32_t soft_budgets[] = { 1u, 2u, 4u, 8u, 16u };
    const uint32_t avbd_budgets[] = { 1u, 2u, 4u, 8u, 16u, 32u };
    printf("%-11s %-11s %6s %8s %9s %8s %8s %8s %8s %8s %8s\n", "scene",
           "solver", "budget", "ms/step", "penetr.mm", "speed", "drift", "drop",
           "stretch", "slide", "jgap.mm");
    for (size_t c = 0; c < sizeof(k_scenes) / sizeof(k_scenes[0]); c++) {
        const rc_scene *scene = &k_scenes[c];
        if (only != NULL && strcmp(only, scene->name) != 0) {
            continue;
        }
        for (uint32_t solver = 0; solver < (uint32_t)RC_SOLVER_COUNT;
             solver++) {
            const bool soft_like =
                (solver == RC_SOFT || solver == RC_SOFT_JOINTS_S ||
                 solver == RC_SOFT_JOINTS_FIRST_S);
            const uint32_t *budgets = soft_like ? soft_budgets : avbd_budgets;
            const uint32_t count =
                soft_like
                    ? (uint32_t)(sizeof(soft_budgets) / sizeof(soft_budgets[0]))
                    : (uint32_t)(sizeof(avbd_budgets) /
                                 sizeof(avbd_budgets[0]));
            for (uint32_t b = 0; b < count; b++) {
                if (quick && b != 2u) {
                    continue;
                }
                const rc_metrics m =
                    rc_run(scene, (rc_solver)solver, budgets[b]);
                printf("%-11s %-11s %6u %8.3f %9.2f %8.3f %8.3f %8.3f %8.4f "
                       "%8.3f %8.2f\n",
                       scene->name, k_solver_names[solver], budgets[b], m.ms,
                       1000.0 * (double)m.penetration, (double)m.speed,
                       (double)m.drift, (double)m.drop, (double)m.stretch,
                       (double)m.slide, 1000.0 * (double)m.joint_gap);
                fflush(stdout);
            }
        }
    }
    return 0;
}
