#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


class MeshUpdateHandler(FileSystemEventHandler):
    """Triggers callback when a mesh file appears or updates."""
    def __init__(self, callback, valid_exts=(".stl", ".obj", ".ply")):
        super().__init__()
        self.callback = callback
        self.valid_exts = valid_exts

    def on_modified(self, event):
        if event.is_directory:
            return
        if event.src_path.endswith(self.valid_exts):
            self.callback(event.src_path)

    def on_created(self, event):
        self.on_modified(event)


class MeshVisualizer(Node):
    def __init__(self):
        super().__init__('mesh_visualizer')

        # === Hardcoded defaults ===
        default_mesh_dir = '/home/lab/robot/meshes'
        default_frame_id = 'ur20_base_link'

        # === Declare parameters (still overridable if needed) ===
        self.declare_parameter('mesh_dir', default_mesh_dir)
        self.declare_parameter('frame_id', default_frame_id)

        self.mesh_dir = self.get_parameter('mesh_dir').value
        self.frame_id = self.get_parameter('frame_id').value

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # === File watcher ===
        self.event_handler = MeshUpdateHandler(self.publish_latest_mesh)
        self.observer = Observer()
        self.observer.schedule(self.event_handler, self.mesh_dir, recursive=False)
        self.observer.start()

        self.get_logger().info("=== Mesh Visualizer Node Started ===")
        self.get_logger().info(f"📁 Watching directory: {self.mesh_dir}")
        self.get_logger().info(f"🗺️  Publishing in frame: {self.frame_id}")

        # Publish the most recent mesh on startup
        self.publish_latest_mesh(None, initial=True)

    def publish_latest_mesh(self, file_path=None, initial=False):
        """Publishes the newest mesh file as a visualization marker."""
        try:
            # Find newest file if not specified
            if file_path is None or initial:
                files = [
                    os.path.join(self.mesh_dir, f)
                    for f in os.listdir(self.mesh_dir)
                    if f.endswith(('.stl', '.obj', '.ply'))
                ]
                if not files:
                    if initial:
                        self.get_logger().warn("No mesh files found yet.")
                    return
                file_path = max(files, key=os.path.getmtime)

            abs_path = os.path.abspath(file_path)
            if not os.path.exists(abs_path):
                self.get_logger().error(f"Mesh file not found: {abs_path}")
                return

            marker = Marker()
            marker.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
            marker.ns = 'reconstructed_mesh'
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = f"file:///{abs_path}"
            marker.pose = Pose()  # identity pose
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color.r = marker.color.g = marker.color.b = 0.8
            marker.color.a = 1.0

            self.marker_pub.publish(marker)
            self.get_logger().info(f"✅ Published mesh ({self.frame_id}): {abs_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to publish mesh: {e}")

    def destroy_node(self):
        self.observer.stop()
        self.observer.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MeshVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
