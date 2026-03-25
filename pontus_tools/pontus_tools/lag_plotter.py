#!/usr/bin/env python3
"""
Real-time timing debugger for the `cluster_coord` image-to-cluster association.

Plots the timing offsets that directly affect matching YOLO detections to
`/pontus/sonar/clustercloud` and keeps message-age summaries in the text panel.
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


@dataclass
class LagSeries:
    label: str
    color: str
    times_sec: Deque[float]
    values_ms: Deque[float]
    linestyle: str = '-'
    linewidth: float = 3.0


class LagPlotter(Node):
    """Monitor timing lag that can break `cluster_coord` association."""

    def __init__(self) -> None:
        super().__init__('lag_plotter')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic', '/pontus/camera_left/image_raw/compressed'),
                ('yolo_topic', '/pontus/camera_left/yolo_results'),
                ('pointcloud_topic', '/pontus/sonar/clustercloud'),
                ('odom_topic', '/pontus/odometry'),
                ('tf_target_frame', 'odom'),
                ('tf_source_frame', 'base_link'),
                ('history_size', 300),
                ('plot_window_seconds', 30.0),
                ('plot_update_hz', 5.0),
                ('keep_plot_in_background', True),
                ('good_lag_ms', 50.0),
                ('warn_lag_ms', 100.0),
                ('bad_lag_ms', 200.0),
            ],
        )

        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.yolo_topic = str(self.get_parameter('yolo_topic').value)
        self.pointcloud_topic = str(self.get_parameter('pointcloud_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.tf_target_frame = str(self.get_parameter('tf_target_frame').value)
        self.tf_source_frame = str(self.get_parameter('tf_source_frame').value)
        self.history_size = max(20, int(self.get_parameter('history_size').value))
        self.plot_window_seconds = max(
            1.0, float(self.get_parameter('plot_window_seconds').value)
        )
        self.plot_update_hz = max(1.0, float(self.get_parameter('plot_update_hz').value))
        self.keep_plot_in_background = bool(
            self.get_parameter('keep_plot_in_background').value
        )
        self.good_lag_ms = max(1.0, float(self.get_parameter('good_lag_ms').value))
        self.warn_lag_ms = max(self.good_lag_ms, float(self.get_parameter('warn_lag_ms').value))
        self.bad_lag_ms = max(self.warn_lag_ms, float(self.get_parameter('bad_lag_ms').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_camera_stamp_sec: Optional[float] = None
        self.last_pointcloud_stamp_sec: Optional[float] = None
        self.last_odom_stamp_sec: Optional[float] = None
        self.last_yolo_stamp_sec: Optional[float] = None
        self.last_exact_tf_stamp_sec: Optional[float] = None
        self.last_latest_tf_stamp_sec: Optional[float] = None
        self.tf_exact_success_count = 0
        self.tf_exact_failure_count = 0
        self.tf_latest_success_count = 0
        self.tf_latest_failure_count = 0
        self.tf_future_extrapolation_count = 0
        self.last_tf_error = 'waiting for TF lookup'
        self.plot_closed = False
        self.start_time_sec = self._now_seconds()

        self.age_series: Dict[str, LagSeries] = {
            'camera': self._make_series('Camera age', 'tab:blue'),
            'yolo': self._make_series('YOLO age', 'tab:orange'),
            'pointcloud': self._make_series('Clustercloud age', 'tab:purple'),
            'odom': self._make_series('Odom age', 'tab:green'),
            'tf': self._make_series('Camera TF age', 'tab:red'),
        }
        self.offset_series: Dict[str, LagSeries] = {
            'yolo_pointcloud': self._make_series('YOLO - clustercloud', 'tab:red'),
            'camera_pointcloud': self._make_series('Camera - clustercloud', 'tab:blue'),
            'odom_pointcloud': self._make_series(
                'Odom - clustercloud',
                'tab:orange',
                linestyle=':',
                linewidth=2.5,
            ),
            'yolo_latest_tf': self._make_series(
                'YOLO - latest TF',
                'black',
                linestyle='--',
                linewidth=2.5,
            ),
            'latest_tf_pointcloud': self._make_series(
                'Latest TF - clustercloud',
                'tab:green',
                linestyle='--',
                linewidth=2.5,
            ),
            'exact_tf_pointcloud': self._make_series(
                'Exact TF@YOLO - clustercloud',
                'tab:green',
                linestyle='-',
                linewidth=3.5,
            ),
        }
        self.latest_annotations = []

        self._create_subscriptions()
        self._create_plot()
        self.plot_timer = self.create_timer(1.0 / self.plot_update_hz, self.update_plot)

        backend = matplotlib.get_backend().lower()
        if 'agg' in backend:
            self.get_logger().warn(
                'Matplotlib is using a non-interactive backend, so no live GUI window '
                'will appear until you switch to an interactive backend.'
            )

        self.get_logger().info(
            'Lag plotter listening to '
            f'camera="{self.camera_topic}", '
            f'yolo="{self.yolo_topic}", '
            f'pointcloud="{self.pointcloud_topic}", '
            f'odom="{self.odom_topic}", '
            f'tf="{self.tf_target_frame} <- {self.tf_source_frame}".'
        )

    def _make_series(
        self,
        label: str,
        color: str,
        linestyle: str = '-',
        linewidth: float = 3.0,
    ) -> LagSeries:
        return LagSeries(
            label=label,
            color=color,
            times_sec=deque(maxlen=self.history_size),
            values_ms=deque(maxlen=self.history_size),
            linestyle=linestyle,
            linewidth=linewidth,
        )

    def _create_subscriptions(self) -> None:
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
        )

        self.camera_sub = self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self.camera_callback,
            best_effort_qos,
        )
        self.yolo_sub = self.create_subscription(
            Detection2DArray,
            self.yolo_topic,
            self.yolo_callback,
            reliable_qos,
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            best_effort_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            reliable_qos,
        )

    def _create_plot(self) -> None:
        plt.ion()
        if self.keep_plot_in_background:
            matplotlib.rcParams['figure.raise_window'] = False
        self.figure, self.offset_ax = plt.subplots(1, 1, figsize=(12, 8))
        self.figure.canvas.mpl_connect('close_event', self._on_plot_closed)

        self.offset_lines = {}
        for key, lag_series in self.offset_series.items():
            line, = self.offset_ax.plot(
                [],
                [],
                label=lag_series.label,
                color=lag_series.color,
                linewidth=lag_series.linewidth,
                linestyle=lag_series.linestyle,
            )
            self.offset_lines[key] = line

        self.offset_ax.set_title('cluster_coord Association Lag')
        self.offset_ax.set_xlabel('Elapsed time [s]')
        self.offset_ax.set_ylabel('Lag [ms]  (positive = first topic is newer)')
        self.offset_ax.grid(True, alpha=0.3)
        self.offset_ax.legend(loc='upper left')
        self._draw_risk_bands()
        self.offset_ax.axhline(0.0, color='black', linestyle='--', linewidth=1.5, alpha=0.8)

        self.stats_text = self.figure.text(
            0.01,
            0.01,
            '',
            family='monospace',
            fontsize=10,
            va='bottom',
        )

        self.figure.tight_layout(rect=(0.0, 0.18, 1.0, 1.0))
        self.figure.show()
        self._keep_window_in_background()

    def _draw_risk_bands(self) -> None:
        self.offset_ax.axhspan(
            -self.good_lag_ms,
            self.good_lag_ms,
            color='green',
            alpha=0.08,
        )
        self.offset_ax.axhspan(
            self.good_lag_ms,
            self.warn_lag_ms,
            color='gold',
            alpha=0.10,
        )
        self.offset_ax.axhspan(
            -self.warn_lag_ms,
            -self.good_lag_ms,
            color='gold',
            alpha=0.10,
        )
        self.offset_ax.axhspan(
            self.warn_lag_ms,
            self.bad_lag_ms,
            color='orange',
            alpha=0.10,
        )
        self.offset_ax.axhspan(
            -self.bad_lag_ms,
            -self.warn_lag_ms,
            color='orange',
            alpha=0.10,
        )
        self.offset_ax.axhline(self.good_lag_ms, color='green', linestyle=':', linewidth=1.0)
        self.offset_ax.axhline(-self.good_lag_ms, color='green', linestyle=':', linewidth=1.0)
        self.offset_ax.axhline(self.warn_lag_ms, color='orange', linestyle=':', linewidth=1.0)
        self.offset_ax.axhline(-self.warn_lag_ms, color='orange', linestyle=':', linewidth=1.0)
        self.offset_ax.axhline(self.bad_lag_ms, color='red', linestyle=':', linewidth=1.0)
        self.offset_ax.axhline(-self.bad_lag_ms, color='red', linestyle=':', linewidth=1.0)

    def _keep_window_in_background(self) -> None:
        """Best-effort request to keep the matplotlib window from stealing focus."""
        if not self.keep_plot_in_background:
            return

        manager = getattr(self.figure.canvas, 'manager', None)
        window = getattr(manager, 'window', None)
        if window is None:
            return

        try:
            if hasattr(window, 'attributes'):
                window.attributes('-topmost', False)
        except Exception:
            pass

        try:
            if hasattr(window, 'lower'):
                window.lower()
        except Exception:
            pass

        try:
            if hasattr(window, 'showMinimized'):
                window.showMinimized()
                window.showNormal()
        except Exception:
            pass

    def _on_plot_closed(self, _event) -> None:
        self.plot_closed = True

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + (float(stamp.nanosec) / 1e9)

    def _record_series(self, lag_series: LagSeries, event_time_sec: float, value_ms: float) -> None:
        lag_series.times_sec.append(event_time_sec)
        lag_series.values_ms.append(value_ms)

    def _record_age(self, key: str, stamp_sec: float, now_sec: float) -> None:
        self._record_series(self.age_series[key], now_sec, (now_sec - stamp_sec) * 1000.0)

    def camera_callback(self, msg: CompressedImage) -> None:
        stamp_sec = self._stamp_to_seconds(msg.header.stamp)
        if stamp_sec <= 0.0:
            return

        now_sec = self._now_seconds()
        self.last_camera_stamp_sec = stamp_sec
        self._record_age('camera', stamp_sec, now_sec)

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        stamp_sec = self._stamp_to_seconds(msg.header.stamp)
        if stamp_sec <= 0.0:
            return

        now_sec = self._now_seconds()
        self.last_pointcloud_stamp_sec = stamp_sec
        self._record_age('pointcloud', stamp_sec, now_sec)

    def odom_callback(self, msg: Odometry) -> None:
        stamp_sec = self._stamp_to_seconds(msg.header.stamp)
        if stamp_sec <= 0.0:
            return

        now_sec = self._now_seconds()
        self.last_odom_stamp_sec = stamp_sec
        self._record_age('odom', stamp_sec, now_sec)

    def yolo_callback(self, msg: Detection2DArray) -> None:
        yolo_stamp_sec = self._stamp_to_seconds(msg.header.stamp)
        if yolo_stamp_sec <= 0.0:
            return

        now_sec = self._now_seconds()
        self.last_yolo_stamp_sec = yolo_stamp_sec
        self._record_age('yolo', yolo_stamp_sec, now_sec)

        if self.last_pointcloud_stamp_sec is not None:
            self._record_series(
                self.offset_series['yolo_pointcloud'],
                now_sec,
                (yolo_stamp_sec - self.last_pointcloud_stamp_sec) * 1000.0,
            )
        if (
            self.last_camera_stamp_sec is not None
            and self.last_pointcloud_stamp_sec is not None
        ):
            self._record_series(
                self.offset_series['camera_pointcloud'],
                now_sec,
                (self.last_camera_stamp_sec - self.last_pointcloud_stamp_sec) * 1000.0,
            )
        if (
            self.last_odom_stamp_sec is not None
            and self.last_pointcloud_stamp_sec is not None
        ):
            self._record_series(
                self.offset_series['odom_pointcloud'],
                now_sec,
                (self.last_odom_stamp_sec - self.last_pointcloud_stamp_sec) * 1000.0,
            )

        try:
            exact_tf_msg = self.tf_buffer.lookup_transform(
                self.tf_target_frame,
                self.tf_source_frame,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05),
            )
            exact_tf_stamp_sec = self._stamp_to_seconds(exact_tf_msg.header.stamp)
            if exact_tf_stamp_sec > 0.0:
                self.last_exact_tf_stamp_sec = exact_tf_stamp_sec
                self.tf_exact_success_count += 1
                self.last_tf_error = ''
                self._record_age('tf', exact_tf_stamp_sec, now_sec)
                if self.last_pointcloud_stamp_sec is not None:
                    self._record_series(
                        self.offset_series['exact_tf_pointcloud'],
                        now_sec,
                        (exact_tf_stamp_sec - self.last_pointcloud_stamp_sec) * 1000.0,
                    )
        except TransformException as exc:
            self.tf_exact_failure_count += 1
            if 'future' in str(exc).lower():
                self.tf_future_extrapolation_count += 1
            self.last_tf_error = str(exc)
            self.get_logger().debug(
                f'TF lookup failed for {self.tf_target_frame} <- {self.tf_source_frame}: {exc}'
            )

        try:
            latest_tf_msg = self.tf_buffer.lookup_transform(
                self.tf_target_frame,
                self.tf_source_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            latest_tf_stamp_sec = self._stamp_to_seconds(latest_tf_msg.header.stamp)
            if latest_tf_stamp_sec > 0.0:
                self.last_latest_tf_stamp_sec = latest_tf_stamp_sec
                self.tf_latest_success_count += 1
                if self.last_pointcloud_stamp_sec is not None:
                    self._record_series(
                        self.offset_series['latest_tf_pointcloud'],
                        now_sec,
                        (latest_tf_stamp_sec - self.last_pointcloud_stamp_sec) * 1000.0,
                    )
                self._record_series(
                    self.offset_series['yolo_latest_tf'],
                    now_sec,
                    (yolo_stamp_sec - latest_tf_stamp_sec) * 1000.0,
                )
        except TransformException as exc:
            self.tf_latest_failure_count += 1
            if not self.last_tf_error:
                self.last_tf_error = str(exc)

    @staticmethod
    def _format_stamp_line(label: str, stamp_sec: Optional[float]) -> str:
        if stamp_sec is None:
            return f'{label:<18} n/a'
        return f'{label:<18} {stamp_sec:14.3f} s'

    def _risk_label(self, magnitude: float) -> str:
        if magnitude < self.good_lag_ms:
            return 'GOOD'
        if magnitude < self.warn_lag_ms:
            return 'WATCH'
        if magnitude < self.bad_lag_ms:
            return 'HIGH'
        return 'BAD'

    def _build_focus_stats(self) -> List[str]:
        lines = [
            'Association Status',
            f'Zero is ideal. Positive means the first stream is newer. '
            f'GOOD < {self.good_lag_ms:.0f} ms, WATCH < {self.warn_lag_ms:.0f} ms, BAD >= {self.bad_lag_ms:.0f} ms.',
            '',
            'Latest Timestamps',
            self._format_stamp_line('camera', self.last_camera_stamp_sec),
            self._format_stamp_line('yolo', self.last_yolo_stamp_sec),
            self._format_stamp_line('odom', self.last_odom_stamp_sec),
            self._format_stamp_line('clustercloud', self.last_pointcloud_stamp_sec),
            self._format_stamp_line('exact tf@yolo', self.last_exact_tf_stamp_sec),
            self._format_stamp_line('latest tf', self.last_latest_tf_stamp_sec),
            '',
            'Latest Differences',
        ]

        for key in (
            'yolo_pointcloud',
            'camera_pointcloud',
            'odom_pointcloud',
            'yolo_latest_tf',
            'latest_tf_pointcloud',
            'exact_tf_pointcloud',
        ):
            lag_series = self.offset_series[key]
            if not lag_series.values_ms:
                if key == 'exact_tf_pointcloud':
                    lines.append(
                        f'{lag_series.label:<24} waiting for exact TF...'
                    )
                else:
                    lines.append(f'{lag_series.label:<24} waiting for data...')
                continue

            latest = lag_series.values_ms[-1]
            magnitude = abs(latest)
            risk = self._risk_label(magnitude)

            relation = 'newer' if latest >= 0.0 else 'older'
            lines.append(
                f'{lag_series.label:<24} {latest:9.1f} ms  '
                f'({relation})  risk={risk}'
            )

        lines.append('')
        lines.append('TF Lookup Health')
        lines.append(
            f'Exact TF ok={self.tf_exact_success_count} fail={self.tf_exact_failure_count} '
            f'future={self.tf_future_extrapolation_count}'
        )
        lines.append(
            f'Latest TF ok={self.tf_latest_success_count} fail={self.tf_latest_failure_count}'
        )
        if self.last_tf_error:
            lines.append(f'Last TF issue: {self.last_tf_error[:160]}')

        return lines

    def _refresh_latest_annotations(self, latest_elapsed: float) -> None:
        for annotation in self.latest_annotations:
            annotation.remove()
        self.latest_annotations = []

        for key, lag_series in self.offset_series.items():
            if not lag_series.values_ms:
                continue

            latest = lag_series.values_ms[-1]
            annotation = self.offset_ax.annotate(
                f'{latest:.0f} ms',
                xy=(latest_elapsed, latest),
                xytext=(8, 0),
                textcoords='offset points',
                color=lag_series.color,
                fontsize=10,
                fontweight='bold',
                va='center',
                clip_on=False,
            )
            self.latest_annotations.append(annotation)

    def _update_axis(
        self,
        axis,
        line_map: Dict[str, object],
        series_group: Dict[str, LagSeries],
        latest_elapsed: float,
        allow_negative: bool,
    ) -> None:
        window_start = max(0.0, latest_elapsed - self.plot_window_seconds)
        visible_values: List[float] = []

        for key, lag_series in series_group.items():
            x_values = [timestamp - self.start_time_sec for timestamp in lag_series.times_sec]
            y_values = list(lag_series.values_ms)
            line_map[key].set_data(x_values, y_values)

            for x_value, y_value in zip(x_values, y_values):
                if x_value >= window_start:
                    visible_values.append(y_value)

        axis.set_xlim(window_start, max(window_start + 1.0, latest_elapsed + 0.1))
        if not visible_values:
            axis.set_ylim(-100.0 if allow_negative else 0.0, 100.0)
            return

        y_min = min(visible_values)
        y_max = max(visible_values)
        if y_max != y_min:
            padding = max(5.0, (y_max - y_min) * 0.1)
        else:
            padding = abs(y_max) * 0.2 + 5.0

        lower = y_min - padding
        upper = y_max + padding
        if not allow_negative:
            lower = min(0.0, lower)
        axis.set_ylim(lower, upper)

    def update_plot(self) -> None:
        if self.plot_closed:
            self.get_logger().info('Lag plot window closed. Shutting down node.')
            rclpy.shutdown()
            return

        latest_elapsed = max(0.0, self._now_seconds() - self.start_time_sec)
        self._update_axis(
            self.offset_ax,
            self.offset_lines,
            self.offset_series,
            latest_elapsed,
            allow_negative=True,
        )
        self._refresh_latest_annotations(latest_elapsed)

        stats_lines = self._build_focus_stats()
        self.stats_text.set_text('\n'.join(stats_lines))

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        plt.pause(0.001)


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
