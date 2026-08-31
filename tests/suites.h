/* Each suite exposes a runner returning its failure count.
 * Add a declaration here when adding a new suite file. */
#ifndef SILK_SUITES_H
#define SILK_SUITES_H

int sl_core_suite(void);
int sl_math_suite(void);
int sl_shape_suite(void);
int sl_collide_suite(void);
int sl_tree_suite(void);
int sl_body_suite(void);
int sl_world_suite(void);
int sl_step_suite(void);

#endif /* SILK_SUITES_H */
