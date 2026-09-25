# M5 visualization: render env.tscn + the A-path lidar_points cloud, save a PNG.
#
# Demonstrates "Godot visualizes the lidar_points produced by the A-2 path".
# Points file: one "x y z" (Godot world coords) per line (hit points only).
# Run (windowed, needs a display): Godot --path godot_viz ++ --points <f> --out <png>
extends Node3D

var points_path := "points.csv"
var out_png := "viz.png"

func _ready() -> void:
	var args := OS.get_cmdline_user_args()
	var i := 0
	while i < args.size():
		match args[i]:
			"--points": points_path = args[i + 1]; i += 2
			"--out": out_png = args[i + 1]; i += 2
			_: i += 1

	add_child(load("res://env.tscn").instantiate())

	# lighting / ambient so the (shaded) env boxes are visible
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.1, 0.12, 0.15)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.6, 0.6, 0.6)
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, -40, 0)
	add_child(sun)

	# point cloud
	var pts := _load_points(points_path)
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	var box := BoxMesh.new()
	box.size = Vector3(0.04, 0.04, 0.04)
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.2, 0.95, 0.35)
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	box.material = mat
	mm.mesh = box
	mm.instance_count = pts.size()
	for j in pts.size():
		mm.set_instance_transform(j, Transform3D(Basis(), pts[j]))
	var mmi := MultiMeshInstance3D.new()
	mmi.multimesh = mm
	add_child(mmi)

	# camera
	var cam := Camera3D.new()
	cam.position = Vector3(8, 7, 9)
	add_child(cam)
	cam.look_at(Vector3(0, 1.0, 0), Vector3.UP)
	cam.current = true

	print("VIZ: env + ", pts.size(), " points; rendering -> ", out_png)
	call_deferred("_capture")

func _load_points(path: String) -> PackedVector3Array:
	var out := PackedVector3Array()
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		push_error("cannot open points: " + path)
		return out
	while not f.eof_reached():
		var line := f.get_line().strip_edges()
		if line == "":
			continue
		var p := line.split(" ")
		if p.size() >= 3:
			out.append(Vector3(float(p[0]), float(p[1]), float(p[2])))
	f.close()
	return out

func _capture() -> void:
	# let a few frames render before grabbing the framebuffer
	for _k in 3:
		await get_tree().process_frame
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var err := img.save_png(out_png)
	print("VIZ: screenshot save err=", err, " path=", out_png)
	get_tree().quit()
