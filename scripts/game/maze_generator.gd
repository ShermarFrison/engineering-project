## Seeded thin-wall maze on a closed rows x cols grid: randomized DFS (recursive
## backtracker), then each remaining interior wall is removed with probability
## loop_prob to add cycles. The outer border is never removed. Pure.
class_name MazeGenerator
extends RefCounted

const N: int = 1
const S: int = 2
const E: int = 4
const W: int = 8
const ALL: int = N | S | E | W
# Row grows toward +Z (south), column toward +X (east).
const DIRS: Array[Vector2i] = [Vector2i(-1, 0), Vector2i(1, 0), Vector2i(0, 1), Vector2i(0, -1)]
const BITS: Array[int] = [N, S, E, W]
const OPPOSITE: Array[int] = [S, N, W, E]


## Returns rows*cols wall bitmasks, index r * cols + c; a set bit means that wall stands.
static func generate(rng: RandomNumberGenerator, rows: int, cols: int, loop_prob: float = 0.2) -> PackedByteArray:
	var walls := PackedByteArray()
	walls.resize(rows * cols)
	walls.fill(ALL)
	var visited := PackedByteArray()
	visited.resize(rows * cols)
	visited.fill(0)
	var stack: Array[Vector2i] = [Vector2i(0, 0)]
	visited[0] = 1
	while not stack.is_empty():
		var cur: Vector2i = stack[-1]
		var options: Array[int] = []
		for k: int in 4:
			var n: Vector2i = cur + DIRS[k]
			if n.x >= 0 and n.x < rows and n.y >= 0 and n.y < cols and visited[n.x * cols + n.y] == 0:
				options.append(k)
		if options.is_empty():
			stack.pop_back()
			continue
		var d: int = options[rng.randi_range(0, options.size() - 1)]
		var nxt: Vector2i = cur + DIRS[d]
		walls[cur.x * cols + cur.y] &= ~BITS[d]
		walls[nxt.x * cols + nxt.y] &= ~OPPOSITE[d]
		visited[nxt.x * cols + nxt.y] = 1
		stack.append(nxt)
	if loop_prob > 0.0:
		for r: int in rows:
			for c: int in cols:
				var i: int = r * cols + c
				if r + 1 < rows and walls[i] & S and rng.randf() < loop_prob:
					walls[i] &= ~S
					walls[i + cols] &= ~N
				if c + 1 < cols and walls[i] & E and rng.randf() < loop_prob:
					walls[i] &= ~E
					walls[i + 1] &= ~W
	return walls


## XZ footprints of the standing interior walls, collinear neighbours merged into one rect.
## origin is the XZ corner of cell (0, 0); walls are centred on the cell boundaries.
static func interior_wall_rects(walls: PackedByteArray, rows: int, cols: int, origin: Vector2,
		cell: float, thickness: float) -> Array[Rect2]:
	var out: Array[Rect2] = []
	var half: float = thickness * 0.5
	# Horizontal walls: south side of row r, i.e. the line z = origin.y + (r + 1) * cell.
	for r: int in rows - 1:
		var z: float = origin.y + (r + 1) * cell
		var start: int = -1
		for c: int in cols + 1:
			var on: bool = c < cols and walls[r * cols + c] & S
			if on and start < 0:
				start = c
			elif not on and start >= 0:
				out.append(Rect2(origin.x + start * cell - half, z - half, (c - start) * cell + thickness, thickness))
				start = -1
	# Vertical walls: east side of column c, i.e. the line x = origin.x + (c + 1) * cell.
	for c: int in cols - 1:
		var x: float = origin.x + (c + 1) * cell
		var start: int = -1
		for r: int in rows + 1:
			var on: bool = r < rows and walls[r * cols + c] & E
			if on and start < 0:
				start = r
			elif not on and start >= 0:
				out.append(Rect2(x - half, origin.y + start * cell - half, thickness, (r - start) * cell + thickness))
				start = -1
	return out
