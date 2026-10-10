## A named set of RoundConfig variants to run against the same seeds.
class_name ExperimentConfig
extends Resource

@export var variants: Array[RoundConfig] = []
@export var rounds_per_variant: int = 20
