import open3d as o3d
import numpy as np

path = "/home/ysa/workspace/indus-0.15-surface.ply"
output_path = "/home/ysa/workspace/indus-0.15-surface.obj"
pcd = o3d.io.read_point_cloud(path)

o3d.visualization.draw_geometries([pcd])

pcd.estimate_normals()

distance = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distance)
radius = 1.0*avg_dist

# mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         pcd, depth=11)

mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector([radius, radius * 2]))
#o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
mesh.remove_degenerate_triangles()
mesh.remove_duplicated_triangles()
mesh.remove_duplicated_vertices()
mesh.remove_non_manifold_edges()

o3d.io.write_triangle_mesh(output_path, mesh)