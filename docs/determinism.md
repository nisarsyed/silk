# Replay regression contract

`tests/replay.c` compares independently initialized worlds after each committed
operation, using binary32 representations for floats and rejecting non-finite
live state. This is a same-platform, same-build, same-call-sequence contract.
Existing analytic tests retain explicit physical tolerances across platforms;
there is no cross-compiler golden hash or save/restore format.

Comparison covers resolved configuration; body slot generations and packed
ownership; free-list order; every live body scalar/vector and active shape field;
proxy bounds, ids and moved/retry queues; pair-table buckets; ordered contact
handles, materials, manifold normals, features, anchors and cached impulses;
joint slots, descriptors, cached impulses and adjacency; tree roots, allocated
nodes and free-list links. Inactive shape payload, addresses, struct padding,
unused queue tails and solver/query scratch overwritten before use are excluded.
Shape constructor/body normalization tests independently cover inactive-byte
normalization. Step/advance churn uses the same replay comparator; there is no
second partial body-only comparator to maintain.

Failures identify fixture, seed, operation, entity, field and values (float bits
in hex). Deliberate signed-zero and joint-cache perturbations test the comparator
without an intentionally failing suite. Mixed churn has 4,096 operations and a
committed xorshift seed; the coupled contact/joint fixture has 2,000 fixed steps.
Capacity recovery explicitly uses non-touching fat-proxy overlaps so solver
motion cannot hide a lost retry. Reset advances generations; equivalent rebuilds
compare public values and separately reject old handles.

Silk has no public impulse mutation API in 0.3.0: fixtures exercise forces,
point forces, torque and velocity setters, plus contact/joint impulse caches.
Sleeping metadata must join this comparison when #64 lands. #61 should migrate
private access inside this helper; consumers must not adopt its storage access.
