## A named set of RoundConfig variants to run against the same seeds.
class_name ExperimentConfig
extends Resource

@export var variants: Array[RoundConfig] = []
@export var rounds_per_variant: int = 20
## First seed of a run (plus --seed-offset). Tuning used seeds 0-19; reported experiments
## start at 1000 so they never reuse a tuning seed.
@export var seed_base: int = 1000
