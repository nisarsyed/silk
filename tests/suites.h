/* Each suite exposes a runner returning its failure count.
 * Add a declaration here when adding a new suite file. */
#ifndef SILK_SUITES_H
#define SILK_SUITES_H

int sl_core_suite(void);
int sl_math_suite(void);
int sl_energy_suite(void);
int sl_mesh_suite(void);
int sl_solver_suite(void);
int sl_contact_suite(void);
int sl_rigid_suite(void);
int sl_step_suite(void);
int sl_consumer_suite(void);

#endif /* SILK_SUITES_H */
