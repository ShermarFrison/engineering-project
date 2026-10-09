## Two-wheel differential drive (unicycle) model.
## v = r (wl + wr) / 2 ;  w = r (wr - wl) / L
class_name DriveModel
extends RefCounted

var wheel_radius: float = 0.1       # m
var track_width: float = 0.4        # m, distance between wheel contact points
var max_wheel_speed: float = 25.0   # rad/s
var max_wheel_accel: float = 60.0   # rad/s^2

var wl: float = 0.0  # left wheel angular speed, rad/s
var wr: float = 0.0  # right wheel angular speed, rad/s


static func wheels_from_body(v: float, w: float, r: float, track: float) -> Vector2:
	var wl_: float = (v - w * track * 0.5) / r
	var wr_: float = (v + w * track * 0.5) / r
	return Vector2(wl_, wr_)


static func body_from_wheels(wl_: float, wr_: float, r: float, track: float) -> Vector2:
	var v: float = r * (wl_ + wr_) * 0.5
	var w: float = r * (wr_ - wl_) / track
	return Vector2(v, w)


func command(v: float, w: float, dt: float) -> void:
	var target: Vector2 = wheels_from_body(v, w, wheel_radius, track_width)
	target.x = clampf(target.x, -max_wheel_speed, max_wheel_speed)
	target.y = clampf(target.y, -max_wheel_speed, max_wheel_speed)
	var max_delta: float = max_wheel_accel * dt
	wl = move_toward(wl, target.x, max_delta)
	wr = move_toward(wr, target.y, max_delta)


func body_velocity() -> Vector2:
	return body_from_wheels(wl, wr, wheel_radius, track_width)


func max_linear_speed() -> float:
	return wheel_radius * max_wheel_speed
