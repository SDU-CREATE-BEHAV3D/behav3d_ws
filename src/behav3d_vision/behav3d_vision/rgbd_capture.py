from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import Subscriber, ApproximateTimeSynchronizer

class RGBDCapture(Node):
    def __init__(self):
        super().__init__('rgbd_capture')
        ...
        # QoS
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # message_filters subscribers for *images only*
        self.sub_color = Subscriber(self, Image, self.color_topic, qos_profile=img_qos)
        self.sub_depth = Subscriber(self, Image, self.depth_topic, qos_profile=img_qos)

        # plain rclpy subscribers for CameraInfo (no sync)
        self.color_info = None
        self.depth_info = None
        self.create_subscription(
            CameraInfo, self.color_info_topic, self._cb_color_info, info_qos)
        self.create_subscription(
            CameraInfo, self.depth_info_topic, self._cb_depth_info, info_qos)

        # 2-way approx sync (images)
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_color, self.sub_depth],
            queue_size=30, slop=0.10)  # 100 ms slop is forgiving in practice
        self.sync.registerCallback(self.cb_images)

        self.get_logger().info(f"SUBSCRIBING images: {self.color_topic} | {self.depth_topic}")
        self.get_logger().info(f"LISTENING infos:  {self.color_info_topic} | {self.depth_info_topic}")

    def _cb_color_info(self, msg): self.color_info = msg
    def _cb_depth_info(self, msg): self.depth_info = msg

    def cb_images(self, img_msg, depth_msg):
        # use whatever CameraInfo we have most recently
        cinfo_msg = self.color_info
        dinfo_msg = self.depth_info

        color = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        if depth_msg.encoding == '16UC1':
            depth = CvBridge().imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32) / 1000.0
        else:
            depth = CvBridge().imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)

        if self.decimate > 1:
            color = color[::self.decimate, ::self.decimate]
            depth = depth[::self.decimate, ::self.decimate]

        # simple sanity
        mask = (depth > 0.3) & (depth < 3.5) & np.isfinite(depth)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            ci = f"{cinfo_msg.width}x{cinfo_msg.height}" if cinfo_msg else "none"
            di = f"{dinfo_msg.width}x{dinfo_msg.height}" if dinfo_msg else "none"
            self.get_logger().info(
                f"#{self.frame_count} color={color.shape[1]}x{color.shape[0]} "
                f"depth={depth.shape[1]}x{depth.shape[0]} "
                f"info(color={ci}, depth={di}) valid={mask.mean()*100:.1f}%")

        if self.save_every_n > 0 and (self.frame_count % self.save_every_n == 0):
            stamp = img_msg.header.stamp
            t = f"{stamp.sec}_{stamp.nanosec:09d}"
            cv2.imwrite(os.path.join(self.save_dir, f"color_{t}.png"), color)
            depth_u16 = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
            cv2.imwrite(os.path.join(self.save_dir, f"depth_{t}.png"), depth_u16)
