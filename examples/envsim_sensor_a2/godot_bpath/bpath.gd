# Pattern B reference: Godot IntersectRay LiDAR over env.tscn.
#
# Reproduces the Godot-side sensing mechanism (Default3DLiDARController uses the
# same PhysicsRayQueryParameters3D / intersect_ray) on the SAME geometry the
# A-path scans (env.tscn <-> env.xml, single source of truth from obb2godot).
# Emits per-beam ranges so the A-2 (mujoco-sensor) and B (Godot) clouds can be
# compared numerically. Run headless via the non-mono Godot binary.
#
# Grid matches lidar3d_a2_pdu.cpp (channels=17 v[-40,40], width=361 h[-180,180]).
extends Node3D

const N_V := 17
const N_H := 361
const V_LO := -40.0
const V_HI := 40.0
const H_LO := -180.0
const H_HI := 180.0
const MAXD := 20.0
const MIND := 0.05

var pose := Vector3(0, 0, 1)   # MuJoCo/ROS frame (x=North, y=West, z=Up)
var yaw := 0.0
var out_path := "bpath_ranges.csv"
var frames := 0

func _ready() -> void:
	var args := OS.get_cmdline_user_args()
	var i := 0
	while i < args.size():
		match args[i]:
			"--pose":
				pose = Vector3(float(args[i + 1]), float(args[i + 2]), float(args[i + 3]))
				yaw = float(args[i + 4])
				i += 5
			"--out":
				out_path = args[i + 1]
				i += 2
			_:
				i += 1
	add_child(load("res://env.tscn").instantiate())

func _physics_process(_d: float) -> void:
	frames += 1
	if frames < 3:
		return
	var st := get_world_3d().direct_space_state
	# drone origin: MuJoCo (N,W,Up) -> Godot (X=E=-W, Y=Up, Z=N)
	var o := Vector3(-pose.y, pose.z, pose.x)
	var ranges := PackedFloat32Array()
	for iv in N_V:
		var pitch := deg_to_rad(V_LO + (V_HI - V_LO) * float(iv) / float(N_V - 1))
		var ce := cos(pitch)
		var se := sin(pitch)
		for ih in N_H:
			var az := deg_to_rad(H_LO + (H_HI - H_LO) * float(ih) / float(N_H - 1)) + yaw
			var ca := cos(az)
			var sa := sin(az)
			# MuJoCo dir = fwd(N)*ce*ca + left(W)*ce*sa + up*se ; map to Godot
			var dir := Vector3(-ce * sa, se, ce * ca)
			var q := PhysicsRayQueryParameters3D.create(o, o + dir * MAXD)
			var r := st.intersect_ray(q)
			var dist := MAXD
			if not r.is_empty():
				var d := o.distance_to(r.position)
				if d >= MIND and d <= MAXD:
					dist = d
			ranges.append(dist)
	var f := FileAccess.open(out_path, FileAccess.WRITE)
	for v in ranges:
		f.store_line(str(v))
	f.close()
	print("BPATH: wrote ", ranges.size(), " ranges to ", out_path)
	get_tree().quit()
