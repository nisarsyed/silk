# Work and memory diagnostics

`sl_world_get_stats` returns a copy. Counts/capacities describe current storage;
`step` describes the most recently completed valid step; `cumulative` and
occupancy high-water marks start at initialization/reset. An empty valid step
publishes zero workload and its configured substep count. Rejected steps leave
all diagnostics unchanged. Reads do not mutate scratch or invalidate snapshots.

Tree visits count popped query nodes, including AABB rejections. Candidates
count matching leaves before self/duplicate/type/joint/pair filtering. Pair
probes count inspected buckets, including the terminating empty bucket and
back-shift deletion/reinsertion. They count actual implementation work, including
repeated searches. Successful tree create/destroy/reinsert operations increment
proxy counters; merely marking a proxy for retry does not. Contact drops count
failed eligible candidates, matching the existing per-step accessor. The
contact high-water mark is also the pair-occupancy high-water mark.

All work counters saturate at UINT64_MAX. Step work is accumulated independently
of cumulative saturation. Query visits and individual pair-search probes are
counted locally before publication; body type counts piggyback on finalization.
Constraint counts are prepared records, not scalar equations or successful
impulses. Each substep runs joint/contact warm start and biased/relax sweeps;
contact restitution runs once. Consumers can combine these fixed stage semantics
with substep/count values without per-impulse instrumentation.

`sl_world_memory_breakdown_get` uses the same sizing lists as initialization.
Categories include internal struct padding; `padding_bytes` is the alignment
space between arena slices. Categories plus padding equal `arena_bytes` and
`sl_world_memory_bytes`. `world_bytes` is separate caller-owned storage. Invalid
configuration leaves the output unchanged. Joint bytes include adjacency and
constraint scratch and are zero when joints are disabled.

The arena layout/allocation is unchanged. On the measured arm64 ABI, the world
shell grows by 248 bytes (480 to 728): a 184-byte stats record, 56-byte step work,
a one-byte stepping flag and alignment. No per-body diagnostic arrays or step
allocations are added. Per-step cost is bounded by existing loops and capacity;
no diagnostic value changes a simulation decision. Tests pin minimum/maximum
budgets, odd-capacity padding, optional joints and counter saturation. Body
payload is 169 bytes per capacity slot (16 ownership + 32 vectors + 16 rotations
+ 32 scalars + 1 type + 72 shape); padding is additional.
