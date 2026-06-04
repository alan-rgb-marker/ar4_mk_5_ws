import open3d as o3d
import numpy as np
mesh = o3d.io.read_triangle_mesh("/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/model/wheel-holder-no.stl")
mesh.scale(0.001, center=(0, 0, 0))
print("Center:", mesh.get_center())
print("Min bounds:", mesh.get_min_bound())
print("Max bounds:", mesh.get_max_bound())
