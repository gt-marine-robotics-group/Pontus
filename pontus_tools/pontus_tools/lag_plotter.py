#!/usr/bin/env python3
"""
Real-time timing debugger for the `cluster_coord` image-to-cluster association.

What each series measures
──────────────────────────────────────────────────────────────────────────────
YOLO − Camera        : ★ PRIMARY ★  Pure YOLO/model inference latency.
                       yolo_result.stamp − camera_frame.stamp
                       How long from camera capture → YOLO published result.
                       TARGET: <50 ms (TRT)   <200 ms (PyTorch)

YOLO − Clustercloud  : Gap between YOLO detections and sonar clusters at the
                       moment a YOLO result arrives.
                       yolo_result.stamp − latest_clustercloud.stamp
                       cluster_coord uses this to match detections to sonar.
                       TARGET: <100 ms. If large, association will fail.

Camera − Clustercloud: Raw sensor sync offset, independent of YOLO.
                       camera_frame.stamp − latest_clustercloud.stamp
                       Recorded on every camera frame (not just when YOLO fires).
                       TARGET: stable / consistent.

Odom − Clustercloud  : Odometry vs sonar sync.
                       odom.stamp − latest_clustercloud.stamp
                       Relevant for TF transforms used in cluster_coord.
                       TARGET: near-zero / stable.

Exact TF − Clustercloud : Age of the exact camera TF at YOLO stamp vs sonar.
Latest TF − Clustercloud: Age of the most-recent available camera TF vs sonar.
YOLO − Latest TF     : How much newer the YOLO result is than the latest TF.
                       Positive = YOLO is ahead of TF → transforms will lag.
──────────────────────────────────────────────────────────────────────────────
"""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional

import matplotlib
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray


# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class LagSeries:
    label: str
    color: str
    times_sec: Deque[float]
    values_ms: Deque[float]
    linestyle: str = '-'
    linewidth: float = 2.5


# ──────────────────────────────────────────────────────────────────────────────
class LagPlotter(Node):
    """Monitor timing lag that can break cluster_coord association."""

    def __init__(self) -> None:
        super().__init__('lag_plotter')

        self.declare_parameters('', [
            ('camera_topic',             '/pontus/camera_front/image_raw/compressed'),
            ('yolo_topic',               '/pontus/camera_front/yolo_results'),
            ('pointcloud_topic',         '/pontus/sonar/clustercloud'),
            ('odom_topic',               '/pontus/odometry'),
            ('tf_target_frame',          'map'),
            ('tf_source_frame',          'camera_front_optical_frame'),
            ('history_size',             300),
            ('plot_window_seconds',      30.0),
            ('plot_update_hz',           5.0),
            ('keep_plot_in_background',  True),
            ('good_lag_ms',              50.0),
            ('warn_lag_ms',              100.0),
            ('bad_lag_ms',               200.0),
        ])

        self.camera_topic            = str(self.get_parameter('camera_topic').value)
        self.yolo_topic              = str(self.get_parameter('yolo_topic').value)
        self.pointcloud_topic        = str(self.get_parameter('pointcloud_topic').value)
        self.odom_topic              = str(self.get_parameter('odom_topic').value)
        self.tf_target_frame         = str(self.get_parameter('tf_target_frame').value)
        self.tf_source_frame         = str(self.get_parameter('tf_source_frame').value)
        self.history_size            = max(20,  int(self.get_parameter('history_size').value))
        self.plot_window_seconds     = max(1.0, float(self.get_parameter('plot_window_seconds').value))
        self.plot_update_hz          = max(1.0, float(self.get_parameter('plot_update_hz').value))
        self.keep_plot_in_background = bool(self.get_parameter('keep_plot_in_background').value)
        self.good_lag_ms             = max(1.0,              float(self.get_parameter('good_lag_ms').value))
        self.warn_lag_ms             = max(self.good_lag_ms, float(self.get_parameter('warn_lag_ms').value))
        self.bad_lag_ms              = max(self.warn_lag_ms, float(self.get_parameter('bad_lag_ms').value))

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Latest raw stamps (seconds)
        self.last_camera_stamp_sec:     Optional[float] = None
        self.last_pointcloud_stamp_sec: Optional[float] = None
        self.last_odom_stamp_sec:       Optional[float] = None
        self.last_yolo_stamp_sec:       Optional[float] = None
        self.last_exact_tf_stamp_sec:   Optional[float] = None
        self.last_latest_tf_stamp_sec:  Optional[float] = None

        # TF health counters
        self.tf_exact_success_count        = 0
        self.tf_exact_failure_count        = 0
        self.tf_latest_success_count       = 0
        self.tf_latest_failure_count       = 0
        self.tf_future_extrapolation_count = 0
        self.last_tf_error                 = 'waiting for TF lookup'

        self.plot_closed    = False
        self.start_time_sec = self._now_seconds()
        self.latest_annotations: list = []

        # ── series (★ PRIMARY is thicker / rendered last so it's on top) ──────
        self.offset_series: Dict[str, LagSeries] = {
            'yolo_pointcloud':      self._make_series('YOLO − Clustercloud',          'tab:red',    linewidth=2.5),
            'camera_pointcloud':    self._make_series('Camera − Clustercloud',         'tab:blue',   linewidth=2.5),
            'odom_pointcloud':      self._make_series('Odom − Clustercloud',           'tab:orange', linewidth=2.0, linestyle=':'),
            'exact_tf_pointcloud':  self._make_series('Exact TF@YOLO − Clustercloud', 'tab:green',  linewidth=3.0),
            'latest_tf_pointcloud': self._make_series('Latest TF − Clustercloud',     'tab:green',  linewidth=2.0, linestyle='--'),
            'yolo_latest_tf':       self._make_series('YOLO − Latest TF',             'black',      linewidth=2.0, linestyle='--'),
            # PRIMARY last so it renders on top
            'yolo_camera':          self._make_series('YOLO − Camera  ★',             'tab:cyan',   linewidth=4.5),
        }

        self._create_subscriptions()
        self._create_plot()
        self.plot_timer = self.create_timer(1.0 / self.plot_update_hz, self.update_plot)

        if 'agg' in matplotlib.get_backend().lower():
            self.get_logger().warn(
                'Matplotlib non-interactive backend — no live window will appear.')

        self.get_logger().info(
            f'LagPlotter ready | '
            f'camera="{self.camera_topic}" yolo="{self.yolo_topic}" '
            f'pointcloud="{self.pointcloud_topic}" '
            f'tf="{self.tf_target_frame} ← {self.tf_source_frame}"'
        )

    # ── helpers ───────────────────────────────────────────────────────────────
    def _make_series(self, label: str, color: str,
                     linestyle: str = '-', linewidth: float = 2.5) -> LagSeries:
        return LagSeries(
            label=label, color=color,
            times_sec=deque(maxlen=self.history_size),
            values_ms=deque(maxlen=self.history_size),
            linestyle=linestyle, linewidth=linewidth,
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _record(self, key: str, now_sec: float, value_ms: float) -> None:
        self.offset_series[key].times_sec.append(now_sec)
        self.offset_series[key].values_ms.append(value_ms)

    # ── subscriptions ─────────────────────────────────────────────────────────
    def _create_subscriptions(self) -> None:
        best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(CompressedImage,  self.camera_topic,     self.camera_callback,     best_effort)
        self.create_subscription(Detection2DArray, self.yolo_topic,       self.yolo_callback,       reliable)
        self.create_subscription(PointCloud2,      self.pointcloud_topic, self.pointcloud_callback, best_effort)
        self.create_subscription(Odometry,         self.odom_topic,       self.odom_callback,       reliable)

    # ── callbacks ─────────────────────────────────────────────────────────────
    def camera_callback(self, msg: CompressedImage) -> None:
        stamp = self._stamp_to_seconds(msg.header.stamp)
        if stamp <= 0.0:
            return
        self.last_camera_stamp_sec = stamp
        # Record camera-pointcloud on every camera frame (not just when YOLO fires)
        if self.last_pointcloud_stamp_sec is not None:
            self._record('camera_pointcloud', self._now_seconds(),
                         (stamp - self.last_pointcloud_stamp_sec) * 1000.0)

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        stamp = self._stamp_to_seconds(msg.header.stamp)
        if stamp <= 0.0:
            return
        self.last_pointcloud_stamp_sec = stamp

    def odom_callback(self, msg: Odometry) -> None:
        stamp = self._stamp_to_seconds(msg.header.stamp)
        if stamp <= 0.0:
            return
        self.last_odom_stamp_sec = stamp
        if self.last_pointcloud_stamp_sec is not None:
            self._record('odom_pointcloud', self._now_seconds(),
                         (stamp - self.last_pointcloud_stamp_sec) * 1000.0)

    def yolo_callback(self, msg: Detection2DArray) -> None:
        yolo_stamp = self._stamp_to_seconds(msg.header.stamp)
        if yolo_stamp <= 0.0:
            return

        now = self._now_seconds()
        self.last_yolo_stamp_sec = yolo_stamp

        # ★ PRIMARY: camera → YOLO inference latency
        if self.last_camera_stamp_sec is not None:
            self._record('yolo_camera', now,
                         (yolo_stamp - self.last_camera_stamp_sec) * 1000.0)

        # sonar association gap
        if self.last_pointcloud_stamp_sec is not None:
            self._record('yolo_pointcloud', now,
                         (yolo_stamp - self.last_pointcloud_stamp_sec) * 1000.0)

        # TF exact lookup at YOLO stamp
        try:
            exact_tf = self.tf_buffer.lookup_transform(
                self.tf_target_frame, self.tf_source_frame,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05),
            )
            exact_stamp = self._stamp_to_seconds(exact_tf.header.stamp)
            if exact_stamp > 0.0:
                self.last_exact_tf_stamp_sec = exact_stamp
                self.tf_exact_success_count += 1
                self.last_tf_error = ''
                if self.last_pointcloud_stamp_sec is not None:
                    self._record('exact_tf_pointcloud', now,
                                 (exact_stamp - self.last_pointcloud_stamp_sec) * 1000.0)
        except TransformException as exc:
            self.tf_exact_failure_count += 1
            if 'future' in str(exc).lower():
                self.tf_future_extrapolation_count += 1
            self.last_tf_error = str(exc)
            self.get_logger().debug(f'Exact TF failed: {exc}')

        # TF latest lookup
        try:
            latest_tf = self.tf_buffer.lookup_transform(
                self.tf_target_frame, self.tf_source_frame,
                Time(), timeout=Duration(seconds=0.05),
            )
            latest_stamp = self._stamp_to_seconds(latest_tf.header.stamp)
            if latest_stamp > 0.0:
                self.last_latest_tf_stamp_sec = latest_stamp
                self.tf_latest_success_count += 1
                if self.last_pointcloud_stamp_sec is not None:
                    self._record('latest_tf_pointcloud', now,
                                 (latest_stamp - self.last_pointcloud_stamp_sec) * 1000.0)
                self._record('yolo_latest_tf', now,
                             (yolo_stamp - latest_stamp) * 1000.0)
        except TransformException as exc:
            self.tf_latest_failure_count += 1
            if not self.last_tf_error:
                self.last_tf_error = str(exc)

    # ── plot setup ────────────────────────────────────────────────────────────
    def _create_plot(self) -> None:
        plt.ion()
        if self.keep_plot_in_background:
            matplotlib.rcParams['figure.raise_window'] = False

        self.figure, self.offset_ax = plt.subplots(figsize=(13, 7))
        self.figure.canvas.mpl_connect('close_event',
                                       lambda _: setattr(self, 'plot_closed', True))

        self.offset_lines: Dict[str, object] = {}
        for key, s in self.offset_series.items():
            line, = self.offset_ax.plot(
                [], [], label=s.label, color=s.color,
                linewidth=s.linewidth, linestyle=s.linestyle,
            )
            self.offset_lines[key] = line

        self.offset_ax.set_title(
            'cluster_coord Association Lag  (★ YOLO−Camera = inference latency)',
            fontsize=12)
        self.offset_ax.set_xlabel('Elapsed time [s]')
        self.offset_ax.set_ylabel('Offset [ms]  (positive = first topic is newer)')
        self.offset_ax.axhline(0.0, color='black', linestyle='--', linewidth=1.0, alpha=0.5)
        self.offset_ax.grid(True, alpha=0.3)
        self._draw_risk_bands()
        self.offset_ax.legend(loc='upper left', fontsize=8)

        self.stats_text = self.figure.text(
            0.01, 0.01, '', family='monospace', fontsize=8, va='bottom')
        self.figure.tight_layout(rect=(0.0, 0.22, 1.0, 1.0))
        self.figure.show()
        self._push_to_background()

    def _draw_risk_bands(self) -> None:
        for lo, hi, col, alpha in [
            (-self.good_lag_ms,  self.good_lag_ms,  'green',  0.08),
            ( self.good_lag_ms,  self.warn_lag_ms,  'gold',   0.10),
            (-self.warn_lag_ms, -self.good_lag_ms,  'gold',   0.10),
            ( self.warn_lag_ms,  self.bad_lag_ms,   'orange', 0.10),
            (-self.bad_lag_ms,  -self.warn_lag_ms,  'orange', 0.10),
        ]:
            self.offset_ax.axhspan(lo, hi, color=col, alpha=alpha)
        for val, col in [
            ( self.good_lag_ms, 'green'),  (-self.good_lag_ms, 'green'),
            ( self.warn_lag_ms, 'orange'), (-self.warn_lag_ms, 'orange'),
            ( self.bad_lag_ms,  'red'),    (-self.bad_lag_ms,  'red'),
        ]:
            self.offset_ax.axhline(val, color=col, linestyle=':', linewidth=1.0)

    def _push_to_background(self) -> None:
        if not self.keep_plot_in_background:
            return
        manager = getattr(self.figure.canvas, 'manager', None)
        window  = getattr(manager, 'window', None)
        if window is None:
            return
        for method, args in [
            ('attributes', ('-topmost', False)),
            ('lower', ()),
            ('showMinimized', ()),
        ]:
            try:
                fn = getattr(window, method, None)
                if fn:
                    fn(*args)
            except Exception:
                pass

    # ── plot update ───────────────────────────────────────────────────────────
    def _risk_label(self, magnitude: float) -> str:
        if magnitude < self.good_lag_ms:  return 'GOOD '
        if magnitude < self.warn_lag_ms:  return 'WATCH'
        if magnitude < self.bad_lag_ms:   return 'HIGH '
        return 'BAD  '

    @staticmethod
    def _fmt_stamp(label: str, val: Optional[float]) -> str:
        return f'  {label:<24} {"n/a" if val is None else f"{val:14.3f} s"}'

    def _build_stats(self) -> List[str]:
        lines = [
            f'GOOD < {self.good_lag_ms:.0f} ms  |  WATCH < {self.warn_lag_ms:.0f} ms  |  BAD >= {self.bad_lag_ms:.0f} ms  |  positive = first topic is newer',
            '',
            'Latest Stamps',
            self._fmt_stamp('camera',        self.last_camera_stamp_sec),
            self._fmt_stamp('yolo',          self.last_yolo_stamp_sec),
            self._fmt_stamp('clustercloud',  self.last_pointcloud_stamp_sec),
            self._fmt_stamp('odom',          self.last_odom_stamp_sec),
            self._fmt_stamp('exact TF@yolo', self.last_exact_tf_stamp_sec),
            self._fmt_stamp('latest TF',     self.last_latest_tf_stamp_sec),
            '',
            'Offsets (latest)',
        ]
        for s in self.offset_series.values():
            if not s.values_ms:
                lines.append(f'  {s.label:<38} waiting...')
                continue
            v = s.values_ms[-1]
            lines.append(f'  {s.label:<38} {v:+8.1f} ms  {self._risk_label(abs(v))}')

        lines += [
            '',
            f'TF exact  ok={self.tf_exact_success_count} fail={self.tf_exact_failure_count} future={self.tf_future_extrapolation_count}',
            f'TF latest ok={self.tf_latest_success_count} fail={self.tf_latest_failure_count}',
        ]
        if self.last_tf_error:
            lines.append(f'Last TF err: {self.last_tf_error[:120]}')
        return lines

    def _refresh_annotations(self, latest_elapsed: float) -> None:
        for ann in self.latest_annotations:
            ann.remove()
        self.latest_annotations = []
        for s in self.offset_series.values():
            if not s.values_ms:
                continue
            v = s.values_ms[-1]
            ann = self.offset_ax.annotate(
                f'{v:.0f}ms',
                xy=(latest_elapsed, v),
                xytext=(6, 0), textcoords='offset points',
                color=s.color, fontsize=8, fontweight='bold',
                va='center', clip_on=False,
            )
            self.latest_annotations.append(ann)

    def update_plot(self) -> None:
        if self.plot_closed:
            self.get_logger().info('Plot window closed — shutting down.')
            rclpy.shutdown()
            return

        now_elapsed  = max(0.0, self._now_seconds() - self.start_time_sec)
        window_start = max(0.0, now_elapsed - self.plot_window_seconds)
        visible: List[float] = []

        for key, s in self.offset_series.items():
            xs = [t - self.start_time_sec for t in s.times_sec]
            ys = list(s.values_ms)
            self.offset_lines[key].set_data(xs, ys)
            visible.extend(y for x, y in zip(xs, ys) if x >= window_start)

        self.offset_ax.set_xlim(window_start, max(window_start + 1.0, now_elapsed + 0.5))
        if visible:
            pad = max(10.0, (max(visible) - min(visible)) * 0.12)
            self.offset_ax.set_ylim(min(visible) - pad, max(visible) + pad)

        self._refresh_annotations(now_elapsed)
        self.stats_text.set_text('\n'.join(self._build_stats()))
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        plt.pause(0.001)


# ──────────────────────────────────────────────────────────────────────────────
def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LagPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()