import open3d as o3d

path = "/home/ysa/workspace/indus-0.15.ply"
output_path = "/home/ysa/workspace/indus-0.15-alpha.obj"
pcd = o3d.io.read_point_cloud(path)

o3d.visualization.draw_geometries([pcd])

alpha = 0.6
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()

#mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
#o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

o3d.io.write_triangle_mesh(output_path, mesh)