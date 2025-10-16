import open3d as o3d

# Load your existing STL
mesh = o3d.io.read_triangle_mesh("tsdf_mesh_refined.stl")

# Apply vertical translation (Z axis)
mesh.translate((0, 0, 0.2))  # move mesh up by 20 cm

# Save to a new file
o3d.io.write_triangle_mesh("tsdf_mesh_refined_offset.stl", mesh)

print("✅ Saved offset mesh as tsdf_mesh_refined_offset.stl (0.2 m higher)")
