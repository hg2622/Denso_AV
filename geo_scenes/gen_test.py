import sys, time; sys.path.insert(0,'/home/all/carla_ws/carla/PythonAPI/carla'); import carla
xodr = sys.argv[1]; c = carla.Client('localhost', 2000); c.set_timeout(900.0); t = time.time()
w = c.generate_opendrive_world(open(xodr).read(), carla.OpendriveGenerationParameters(vertex_distance=2.0, max_road_length=50.0, wall_height=0.0, additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=True, enable_pedestrian_navigation=False))
print(f'generated in {time.time()-t:.0f}s; waypoints {len(w.get_map().generate_waypoints(10.0))}; spawn pts {len(w.get_map().get_spawn_points())}')
