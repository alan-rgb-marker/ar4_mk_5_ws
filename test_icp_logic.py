import numpy as np

# Mock test for initial guess
x_center, y_center, z_center = 1.0, 2.0, 3.0
cad_center = np.array([0.0, 0.0, 0.1]) # offset origin
R_init = np.eye(3)

t = np.array([x_center, y_center, z_center]) - R_init @ cad_center

init = np.eye(4)
init[:3, :3] = R_init
init[:3, 3] = t

# Apply init to cad_center
transformed_center = init[:3, :3] @ cad_center + init[:3, 3]
print("Transformed center:", transformed_center)
print("Expected:", [x_center, y_center, z_center])
