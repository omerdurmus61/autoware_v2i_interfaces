#!/usr/bin/env python3

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from python_qt_binding import QtCore, QtWidgets
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from v2i_spat_msgs.msg import MovementEvent, SpatPacket


LIGHT_OFF = "#263238"
RED_ON = "#e53935"
YELLOW_ON = "#fdd835"
GREEN_ON = "#43a047"
UNKNOWN_ON = "#90a4ae"
CARD_BG = "#0f172a"
WINDOW_BG = "#e2e8f0"
TEXT_PRIMARY = "#e5e7eb"
TEXT_SECONDARY = "#94a3b8"
TEXT_DARK = "#0f172a"


@dataclass(frozen=True)
class ConfiguredSignal:
    name: str
    description: str
    intersection_id: int
    signal_group: int
    unique_signal_id: int


@dataclass
class SignalSnapshot:
    event_state: int
    remaining_seconds: Optional[float]
    last_update_monotonic: float
    source_moy: int
    source_timestamp_ms: int
    end_offset_deciseconds: Optional[int]


class TrafficLightWidget(QtWidgets.QFrame):
    def __init__(self, signal: ConfiguredSignal, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.signal = signal
        self.event_state = MovementEvent.EVENT_UNKNOWN
        self.remaining_seconds: Optional[float] = None
        self.setObjectName("trafficLightCard")
        self.setMinimumWidth(145)
        self.setMaximumWidth(165)
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        title = QtWidgets.QLabel(self.signal.name)
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setWordWrap(True)
        title.setStyleSheet("font-size: 11px; font-weight: 700; color: #f8fafc;")
        layout.addWidget(title)

        subtitle = QtWidgets.QLabel(
            f"I{self.signal.intersection_id} | SG {self.signal.signal_group}"
        )
        subtitle.setAlignment(QtCore.Qt.AlignCenter)
        subtitle.setStyleSheet("font-size: 9px; color: #cbd5e1;")
        layout.addWidget(subtitle)

        unique_label = QtWidgets.QLabel(f"Unique ID: {self.signal.unique_signal_id}")
        unique_label.setAlignment(QtCore.Qt.AlignCenter)
        unique_label.setStyleSheet("font-size: 9px; color: #94a3b8;")
        layout.addWidget(unique_label)

        self.lamp_box = QtWidgets.QFrame()
        self.lamp_box.setStyleSheet(
            "background-color: #020617; border-radius: 16px; border: 2px solid #334155;"
        )
        lamp_layout = QtWidgets.QVBoxLayout(self.lamp_box)
        lamp_layout.setContentsMargins(12, 12, 12, 12)
        lamp_layout.setSpacing(8)

        self.red_lamp = self._create_lamp()
        self.yellow_lamp = self._create_lamp()
        self.green_lamp = self._create_lamp()

        lamp_layout.addWidget(self.red_lamp, alignment=QtCore.Qt.AlignCenter)
        lamp_layout.addWidget(self.yellow_lamp, alignment=QtCore.Qt.AlignCenter)
        lamp_layout.addWidget(self.green_lamp, alignment=QtCore.Qt.AlignCenter)
        layout.addWidget(self.lamp_box, alignment=QtCore.Qt.AlignCenter)

        self.state_label = QtWidgets.QLabel("Waiting for SPaT")
        self.state_label.setAlignment(QtCore.Qt.AlignCenter)
        self.state_label.setStyleSheet("font-size: 11px; font-weight: 600; color: #f8fafc;")
        layout.addWidget(self.state_label)
        self._apply_state()

    def _create_lamp(self) -> QtWidgets.QLabel:
        lamp = QtWidgets.QLabel("--")
        lamp.setFixedSize(54, 54)
        lamp.setAlignment(QtCore.Qt.AlignCenter)
        lamp.setStyleSheet(self._lamp_stylesheet(LIGHT_OFF, TEXT_PRIMARY))
        return lamp

    def _lamp_stylesheet(self, color: str, text_color: str) -> str:
        return (
            "border-radius: 27px; "
            f"background-color: {color}; "
            "border: 2px solid #1e293b; "
            f"color: {text_color}; "
            "font-size: 10px; font-weight: 700;"
        )

    def update_state(self, event_state: int, remaining_seconds: Optional[float]) -> None:
        self.event_state = event_state
        self.remaining_seconds = remaining_seconds
        self._apply_state()

    def mark_stale(self) -> None:
        self.event_state = MovementEvent.EVENT_UNKNOWN
        self.remaining_seconds = None
        self._apply_state()

    def _countdown_text(self, active: bool) -> str:
        if not active:
            return "--"
        if self.remaining_seconds is None:
            return "?"
        return f"{max(0.0, self.remaining_seconds):.1f}s"

    def _apply_state(self) -> None:
        red_color = LIGHT_OFF
        yellow_color = LIGHT_OFF
        green_color = LIGHT_OFF
        state_name = "Unknown"

        if self.event_state == MovementEvent.STOP_AND_REMAIN:
            red_color = RED_ON
            state_name = "Red"
        elif self.event_state == MovementEvent.PROTECTED_CLEARANCE:
            yellow_color = YELLOW_ON
            state_name = "Yellow"
        elif self.event_state in (
            MovementEvent.PROTECTED_MOVEMENT_ALLOWED,
            MovementEvent.PERMISSIVE_MOVEMENT_ALLOWED,
        ):
            green_color = GREEN_ON
            state_name = "Green"
        else:
            state_name = "Unknown"

        self.red_lamp.setStyleSheet(self._lamp_stylesheet(red_color, TEXT_PRIMARY))
        self.yellow_lamp.setStyleSheet(self._lamp_stylesheet(yellow_color, TEXT_DARK))
        self.green_lamp.setStyleSheet(self._lamp_stylesheet(green_color, TEXT_PRIMARY))

        self.red_lamp.setText(
            self._countdown_text(self.event_state == MovementEvent.STOP_AND_REMAIN)
        )
        self.yellow_lamp.setText(
            self._countdown_text(self.event_state == MovementEvent.PROTECTED_CLEARANCE)
        )
        self.green_lamp.setText(
            self._countdown_text(
                self.event_state
                in (
                    MovementEvent.PROTECTED_MOVEMENT_ALLOWED,
                    MovementEvent.PERMISSIVE_MOVEMENT_ALLOWED,
                )
            )
        )
        self.state_label.setText(state_name)


class SpatVisualizerWindow(QtWidgets.QMainWindow):
    def __init__(self, window_title: str, signals: List[ConfiguredSignal]) -> None:
        super().__init__()
        self.setWindowTitle(window_title)
        self.resize(860, 330)
        self.setMinimumSize(820, 300)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(8)

        header = QtWidgets.QLabel("V2I Traffic Light SPaT Monitor")
        header.setStyleSheet("font-size: 16px; font-weight: 800; color: #0f172a;")
        outer.addWidget(header)

        subheader = QtWidgets.QLabel(
            "Five configured signal groups are visualized with live color and remaining-time updates."
        )
        subheader.setStyleSheet("font-size: 10px; color: #334155;")
        outer.addWidget(subheader)

        content = QtWidgets.QWidget()
        outer.addWidget(content, 1)
        grid = QtWidgets.QGridLayout(content)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)

        self.widgets: Dict[int, TrafficLightWidget] = {}
        for index, signal in enumerate(signals):
            widget = TrafficLightWidget(signal)
            grid.addWidget(widget, 0, index)
            self.widgets[signal.unique_signal_id] = widget

    def update_signal(
        self, unique_signal_id: int, event_state: int, remaining_seconds: Optional[float]
    ) -> None:
        widget = self.widgets.get(unique_signal_id)
        if widget is not None:
            widget.update_state(event_state, remaining_seconds)

    def mark_stale(self, unique_signal_id: int) -> None:
        widget = self.widgets.get(unique_signal_id)
        if widget is not None:
            widget.mark_stale()


class V2ISpatVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("v2i_spat_visualizer")

        default_config = (
            PathJoinHelper.package_share("v2i_spat_visualizer") / "config" / "intersections.yaml"
        )
        self.declare_parameter("config_file", str(default_config))
        self.declare_parameter("input_topic", "/v2i/spat/raw")
        self.declare_parameter("refresh_rate_hz", 10.0)
        self.declare_parameter("stale_timeout_sec", 1.5)
        self.declare_parameter("window_title", "V2I SPaT Visualizer")
        config_file = str(self.get_parameter("config_file").value)
        config = load_visualizer_config(config_file)

        self.input_topic = str(config.get("input_topic", self.get_parameter("input_topic").value))
        self.refresh_rate_hz = float(
            config.get("refresh_rate_hz", self.get_parameter("refresh_rate_hz").value)
        )
        self.stale_timeout_sec = float(
            config.get("stale_timeout_sec", self.get_parameter("stale_timeout_sec").value)
        )
        self.window_title = str(
            config.get("window_title", self.get_parameter("window_title").value)
        )
        self.signals = self._load_signals_from_config(config)
        self.signals_by_key: Dict[Tuple[int, int], ConfiguredSignal] = {
            (signal.intersection_id, signal.signal_group): signal for signal in self.signals
        }
        self.snapshots: Dict[int, SignalSnapshot] = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            SpatPacket, self.input_topic, self.spat_callback, qos
        )

        configured_ids = ", ".join(str(signal.unique_signal_id) for signal in self.signals)
        self.get_logger().info(f"Loaded config file: {config_file}")
        self.get_logger().info(f"Watching unique signal IDs: {configured_ids}")

    def _load_signals_from_config(self, config: Dict) -> List[ConfiguredSignal]:
        raw_signals = config.get("signals", [])
        if not raw_signals:
            raise RuntimeError("No intersections configured for v2i_spat_visualizer.")

        signals: List[ConfiguredSignal] = []
        for index, raw_signal in enumerate(raw_signals):
            if not isinstance(raw_signal, dict):
                raise RuntimeError(f"Configured intersection #{index} is not a map: {raw_signal}")

            intersection_id = int(raw_signal["intersection_id"])
            signal_group = int(raw_signal["signal_group"])
            computed_unique_id = int(f"{intersection_id}{signal_group}")
            configured_unique_id = int(raw_signal.get("unique_signal_id", computed_unique_id))
            if configured_unique_id != computed_unique_id:
                raise RuntimeError(
                    "Configured unique_signal_id does not match concatenation rule for "
                    f"intersection {intersection_id} signal group {signal_group}."
                )

            signals.append(
                ConfiguredSignal(
                    name=str(raw_signal.get("name", f"Intersection {intersection_id} SG{signal_group}")),
                    description=str(
                        raw_signal.get(
                            "description",
                            "Configured watched signal group for SPaT monitoring.",
                        )
                    ),
                    intersection_id=intersection_id,
                    signal_group=signal_group,
                    unique_signal_id=configured_unique_id,
                )
                )

        return signals

    def choose_primary_event(self, events: List[MovementEvent]) -> Optional[MovementEvent]:
        supported: List[Tuple[int, MovementEvent]] = []
        fallback: List[MovementEvent] = []

        for event in events:
            if event.event_state == MovementEvent.EVENT_UNKNOWN:
                continue

            if event.timing.has_max_end_time:
                supported.append((int(event.timing.max_end_time), event))
            elif event.timing.has_min_end_time:
                supported.append((int(event.timing.min_end_time), event))
            else:
                fallback.append(event)

        if supported:
            supported.sort(key=lambda item: item[0])
            return supported[0][1]
        if fallback:
            return fallback[0]
        return events[0] if events else None

    def _predict_end_time_from_guide(
        self, moy: int, timestamp_ms: int, event: MovementEvent
    ) -> Tuple[Optional[datetime], Optional[int]]:
        offset_deciseconds: Optional[int] = None
        if event.timing.has_max_end_time:
            offset_deciseconds = int(event.timing.max_end_time)
        elif event.timing.has_min_end_time:
            offset_deciseconds = int(event.timing.min_end_time)

        if offset_deciseconds is None:
            return None, None

        current_year = datetime.now(timezone.utc).year
        year_start = datetime(current_year, 1, 1, tzinfo=timezone.utc)
        source_time = year_start + timedelta(minutes=int(moy), milliseconds=int(timestamp_ms))
        predicted_end_time = source_time + timedelta(seconds=offset_deciseconds / 10.0)
        return predicted_end_time, offset_deciseconds

    def _remaining_seconds_from_prediction(
        self, predicted_end_time: Optional[datetime], offset_deciseconds: Optional[int]
    ) -> Optional[float]:
        if predicted_end_time is None or offset_deciseconds is None:
            return None

        remaining_seconds = (predicted_end_time - datetime.now(timezone.utc)).total_seconds()
        if remaining_seconds < -1.0 or remaining_seconds > 3600.0:
            return offset_deciseconds / 10.0
        return max(0.0, remaining_seconds)

    def spat_callback(self, msg: SpatPacket) -> None:
        receipt_time = self.get_clock().now().nanoseconds / 1e9

        for intersection in msg.spat.intersections:
            intersection_id = int(intersection.intersection_id)
            source_moy = int(intersection.moy)
            source_timestamp_ms = int(intersection.time_stamp)

            for state in intersection.states:
                signal_group = int(state.signal_group)
                configured_signal = self.signals_by_key.get((intersection_id, signal_group))
                if configured_signal is None or not state.events:
                    continue

                primary_event = self.choose_primary_event(list(state.events))
                if primary_event is None:
                    continue

                predicted_end_time, offset_deciseconds = self._predict_end_time_from_guide(
                    source_moy, source_timestamp_ms, primary_event
                )
                remaining_seconds = self._remaining_seconds_from_prediction(
                    predicted_end_time, offset_deciseconds
                )

                self.snapshots[configured_signal.unique_signal_id] = SignalSnapshot(
                    event_state=int(primary_event.event_state),
                    remaining_seconds=remaining_seconds,
                    last_update_monotonic=receipt_time,
                    source_moy=source_moy,
                    source_timestamp_ms=source_timestamp_ms,
                    end_offset_deciseconds=offset_deciseconds,
                )


class PathJoinHelper:
    @staticmethod
    def package_share(package_name: str):
        from pathlib import Path

        return Path(get_package_share_directory(package_name))


def load_visualizer_config(config_path: str) -> Dict:
    with open(config_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data or {}


def build_stylesheet() -> str:
    return f"""
        QMainWindow {{
            background-color: {WINDOW_BG};
        }}
        QScrollArea {{
            background: transparent;
        }}
        QFrame#trafficLightCard {{
            background-color: {CARD_BG};
            border-radius: 24px;
            border: 1px solid #1e293b;
        }}
    """


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = V2ISpatVisualizer()

    app = QtWidgets.QApplication([])
    app.setStyle("Fusion")
    app.setStyleSheet(build_stylesheet())

    window = SpatVisualizerWindow(node.window_title, node.signals)
    window.show()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ros_spin_timer = QtCore.QTimer()
    ros_spin_timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0.0))
    ros_spin_timer.start(max(1, int(1000.0 / max(1.0, node.refresh_rate_hz))))

    ui_refresh_timer = QtCore.QTimer()

    def refresh_widgets() -> None:
        current_monotonic = node.get_clock().now().nanoseconds / 1e9
        for signal in node.signals:
            snapshot = node.snapshots.get(signal.unique_signal_id)
            if snapshot is None:
                window.mark_stale(signal.unique_signal_id)
                continue

            if (current_monotonic - snapshot.last_update_monotonic) > node.stale_timeout_sec:
                window.mark_stale(signal.unique_signal_id)
                continue

            remaining_seconds = snapshot.remaining_seconds
            if remaining_seconds is not None:
                elapsed = current_monotonic - snapshot.last_update_monotonic
                remaining_seconds = max(0.0, remaining_seconds - elapsed)

            window.update_signal(
                signal.unique_signal_id, snapshot.event_state, remaining_seconds
            )

    ui_refresh_timer.timeout.connect(refresh_widgets)
    ui_refresh_timer.start(max(1, int(1000.0 / max(1.0, node.refresh_rate_hz))))

    try:
        app.exec_()
    finally:
        ros_spin_timer.stop()
        ui_refresh_timer.stop()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
