import json

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from detector import Params, Pipeline
from detector.pointcloud import structured, valid_xyz


def track_to_sensor(res, calib, s, d, h):
    s = np.asarray(s, dtype=float)
    slh = np.stack([s, res.path.center(s) + d, res.path.floor(s) + h], axis=1)
    return calib.to_sensor(slh)


def cloud_msg(header, xyz):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(xyz)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(xyz)
    msg.is_dense = True
    msg.data = np.asarray(xyz, dtype=np.float32).tobytes()
    return msg


class DetectorNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.declare_parameter('input_topic', '')
        self.declare_parameter('config', '')
        self.declare_parameter('log_path', '')
        self.declare_parameter('publish_debug', True)
        config = self.get_parameter('config').value or None
        self.params = Params.load(config)
        self.pipeline = Pipeline(self.params)
        self.debug = self.get_parameter('publish_debug').value
        log_path = self.get_parameter('log_path').value
        self.log = open(log_path, 'w', encoding='utf-8') if log_path else None
        self.qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = None
        self.pub_status = self.create_publisher(String, '/obstacle/status', 10)
        self.pub_detected = self.create_publisher(Bool, '/obstacle/detected', 10)
        self.pub_distance = self.create_publisher(Float32, '/obstacle/distance', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/obstacle/markers', 10)
        self.pub_points = self.create_publisher(PointCloud2, '/obstacle/points', 10)
        self.frames = 0
        topic = self.get_parameter('input_topic').value
        if topic:
            self.subscribe(topic)
        else:
            self.get_logger().info('waiting for a PointCloud2 topic')
            self.discovery = self.create_timer(0.5, self.discover)

    def subscribe(self, topic):
        self.sub = self.create_subscription(PointCloud2, topic, self.on_cloud, self.qos)
        self.get_logger().info(f'listening on {topic}')

    def discover(self):
        for name, types in self.get_topic_names_and_types():
            if 'sensor_msgs/msg/PointCloud2' in types and not name.startswith('/obstacle'):
                self.discovery.cancel()
                self.subscribe(name)
                return

    def on_cloud(self, msg):
        arr = structured(msg.fields, msg.point_step, msg.data, msg.width * msg.height)
        xyz = valid_xyz(arr)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        res = self.pipeline.process(xyz, stamp)
        self.frames += 1
        summary = res.summary()
        summary['frame'] = self.frames
        text = json.dumps(summary, ensure_ascii=False)
        self.pub_status.publish(String(data=text))
        self.pub_detected.publish(Bool(data=res.obstacle))
        self.pub_distance.publish(Float32(data=float(res.nearest) if res.obstacle else -1.0))
        if self.log:
            self.log.write(text + '\n')
            self.log.flush()
        if res.obstacle:
            self.get_logger().warn(f'obstacle at {res.nearest:.1f} m, objects: {len(res.tracks)}')
        if self.debug:
            self.publish_debug(msg.header, res)

    def publish_debug(self, header, res):
        calib = self.pipeline.calib
        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        s = np.arange(self.params.min_range, max(res.visibility, self.params.min_range + 1), 2.0)
        half = self.params.gauge_half_width
        top = self.params.gauge_height
        for i, (d, h) in enumerate(((-half, 0.0), (half, 0.0), (-half, top), (half, top))):
            m = Marker()
            m.header = header
            m.ns = 'gauge'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.scale.x = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.8, 1.0, 0.8
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in track_to_sensor(res, calib, s, d, h)]
            markers.markers.append(m)
        for tr in res.tracks:
            sc = tr.distance + tr.size[0] / 2
            c = track_to_sensor(res, calib, [sc], tr.lateral, tr.height - tr.size[2] / 2)[0]
            box = Marker()
            box.header = header
            box.ns = 'obstacles'
            box.id = tr.id
            box.type = Marker.CUBE
            box.pose.position = Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))
            box.pose.orientation.w = 1.0
            box.scale.x = max(tr.size[1], 0.3)
            box.scale.y = max(tr.size[0], 0.3)
            box.scale.z = max(tr.size[2], 0.3)
            box.color.r, box.color.g, box.color.b, box.color.a = 1.0, 0.1, 0.1, 0.6
            markers.markers.append(box)
            label = Marker()
            label.header = header
            label.ns = 'labels'
            label.id = tr.id
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position = Point(x=float(c[0]), y=float(c[1]), z=float(c[2]) + 1.5)
            label.pose.orientation.w = 1.0
            label.scale.z = 1.0
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
            label.text = f'{tr.distance:.1f} m'
            markers.markers.append(label)
        self.pub_markers.publish(markers)
        if len(res.candidates):
            c = res.candidates
            self.pub_points.publish(cloud_msg(header, track_to_sensor(res, calib, c[:, 0], c[:, 1], c[:, 2])))


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.log:
            node.log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
