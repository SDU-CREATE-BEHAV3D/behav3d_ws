import open3d as o3d


### visualize T_base_ir vs T_base_tool0 poses
def visualize_camera_poses(T_base_tool0_list, T_base_ir_list):
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    tool0_frames = []
    ir_frames = []
    for T_tool0, T_ir in zip(T_base_tool0_list, T_base_ir_list):
        mesh_tool0 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        mesh_tool0.transform(T_tool0)
        tool0_frames.append(mesh_tool0)

        mesh_ir = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        mesh_ir.transform(T_ir)
        ir_frames.append(mesh_ir)

    o3d.visualization.draw([mesh_frame, *tool0_frames, *ir_frames])