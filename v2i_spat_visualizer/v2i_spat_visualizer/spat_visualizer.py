#!/usr/bin/env python3

from dataclasses import dataclass
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
        lamp_layout.setSpacing(10)

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
        self.setMinimumSize(180, 300)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(8)

        self.viewport = QtWidgets.QFrame()
        self.viewport.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.viewport.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding
        )
        outer.addWidget(self.viewport, 1)

        self.content = QtWidgets.QWidget(self.viewport)
        grid = QtWidgets.QGridLayout(self.content)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)

        self.widgets: Dict[int, TrafficLightWidget] = {}
        for index, signal in enumerate(signals):
            widget = TrafficLightWidget(signal)
            grid.addWidget(widget, 0, index)
            self.widgets[signal.unique_signal_id] = widget

        self.content.setFixedSize(self.content.sizeHint())
        QtCore.QTimer.singleShot(0, self._position_content)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._position_content()

    def _position_content(self) -> None:
        available_width = self.viewport.width()
        # Keep the card strip fixed-size and anchored to the left. When the
        # window narrows, crop from the right edge instead of rescaling.
        x = 0
        y = max(0, (self.viewport.height() - self.content.height()) // 2)
        self.content.move(x, y)

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
        self.declare_parameter("debug_spat_timing", False)
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
        self.debug_spat_timing = bool(
            config.get("debug_spat_timing", self.get_parameter("debug_spat_timing").value)
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
        self.get_logger().info(f"SPaT timing debug: {self.debug_spat_timing}")

    def is_supported_event_state(self, event_state: int) -> bool:
        return event_state in (
            MovementEvent.STOP_AND_REMAIN,
            MovementEvent.PROTECTED_MOVEMENT_ALLOWED,
            MovementEvent.PROTECTED_CLEARANCE,
            MovementEvent.PERMISSIVE_MOVEMENT_ALLOWED,
        )

    def event_end_offset_deciseconds(self, event: MovementEvent) -> Optional[int]:
        return self.select_event_end_offset_deciseconds(event, current_deciseconds=None)

    def current_spat_time_deciseconds(self, moy: int, timestamp_ms: int) -> float:
        """
        Convert the SPaT message's current time reference into the same time axis
        used by event min/max end times: deciseconds within the current hour.
        """
        minute_in_hour = int(moy) % 60
        second_in_minute = float(timestamp_ms) / 1000.0
        return (minute_in_hour * 60.0 + second_in_minute) * 10.0

    def normalize_decisecond_delta(self, delta_deciseconds: float) -> float:
        if delta_deciseconds < 0.0:
            return delta_deciseconds + 36000.0
        return delta_deciseconds

    def raw_decisecond_delta(
        self, end_offset_deciseconds: float, current_deciseconds: float
    ) -> float:
        return end_offset_deciseconds - current_deciseconds

    def select_event_end_offset_deciseconds(
        self, event: MovementEvent, current_deciseconds: Optional[float]
    ) -> Optional[int]:
        """
        Choose the more meaningful end target for countdown display.

        Prefer min_end_time by default, but if it is effectively "now" while
        max_end_time still provides a reasonable future horizon, use max_end_time
        instead. This avoids false one-hour wraps and nearly-fixed tiny countdowns.
        """
        has_min = bool(event.timing.has_min_end_time)
        has_max = bool(event.timing.has_max_end_time)
        min_end = int(event.timing.min_end_time) if has_min else None
        max_end = int(event.timing.max_end_time) if has_max else None

        if not has_min and not has_max:
            return None
        if current_deciseconds is None:
            return min_end if has_min else max_end
        if not has_min:
            return max_end
        if not has_max:
            return min_end

        raw_d_min = self.raw_decisecond_delta(float(min_end), current_deciseconds) / 10.0
        raw_d_max = self.raw_decisecond_delta(float(max_end), current_deciseconds) / 10.0
        d_min = self.normalize_decisecond_delta(
            self.raw_decisecond_delta(float(min_end), current_deciseconds)
        ) / 10.0
        d_max = self.normalize_decisecond_delta(
            self.raw_decisecond_delta(float(max_end), current_deciseconds)
        ) / 10.0

        min_is_effectively_now = -1.0 <= raw_d_min <= 1.0
        max_is_reasonable_future = 1.0 < d_max <= 120.0
        max_is_not_effectively_now = raw_d_max > 1.0

        if min_is_effectively_now and max_is_reasonable_future and max_is_not_effectively_now:
            return max_end

        return min_end

    def remaining_seconds_from_spat(
        self, moy: int, timestamp_ms: int, event: MovementEvent
    ) -> Optional[float]:
        current_deciseconds = self.current_spat_time_deciseconds(moy, timestamp_ms)
        end_offset_deciseconds = self.select_event_end_offset_deciseconds(
            event, current_deciseconds
        )
        if end_offset_deciseconds is None:
            return None

        remaining_deciseconds = self.normalize_decisecond_delta(
            float(end_offset_deciseconds) - current_deciseconds
        )

        return max(0.0, remaining_deciseconds / 10.0)

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

    def choose_active_event(self, events: List[MovementEvent]) -> Optional[MovementEvent]:
        """
        Interpret the SPaT event list as an ordered phase sequence.

        The first supported event is treated as the active phase, and its earliest
        available end bound is used as the immediate switch time. If the order is
        not usable, fall back to the supported event with the smallest end time.
        """
        if not events:
            return None

        supported_in_order: List[MovementEvent] = [
            event for event in events if self.is_supported_event_state(int(event.event_state))
        ]
        if not supported_in_order:
            return events[0]

        ordered_candidate = supported_in_order[0]
        if self.event_end_offset_deciseconds(ordered_candidate) is not None:
            return ordered_candidate

        timed_candidates: List[Tuple[int, MovementEvent]] = []
        for event in supported_in_order:
            end_offset = self.event_end_offset_deciseconds(event)
            if end_offset is not None:
                timed_candidates.append((end_offset, event))

        if timed_candidates:
            timed_candidates.sort(key=lambda item: item[0])
            return timed_candidates[0][1]

        return ordered_candidate

    def describe_event(self, event: MovementEvent) -> str:
        min_end = int(event.timing.min_end_time) if event.timing.has_min_end_time else None
        max_end = int(event.timing.max_end_time) if event.timing.has_max_end_time else None
        return (
            f"state={int(event.event_state)} "
            f"min_end={min_end} "
            f"max_end={max_end}"
        )

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

                active_event = self.choose_active_event(list(state.events))
                if active_event is None:
                    continue

                current_spat_deciseconds = self.current_spat_time_deciseconds(
                    source_moy, source_timestamp_ms
                )
                offset_deciseconds = self.select_event_end_offset_deciseconds(
                    active_event, current_spat_deciseconds
                )
                remaining_seconds = self.remaining_seconds_from_spat(
                    source_moy, source_timestamp_ms, active_event
                )

                if self.debug_spat_timing:
                    min_end = (
                        int(active_event.timing.min_end_time)
                        if active_event.timing.has_min_end_time
                        else None
                    )
                    max_end = (
                        int(active_event.timing.max_end_time)
                        if active_event.timing.has_max_end_time
                        else None
                    )
                    d_min = None
                    d_max = None
                    if min_end is not None:
                        raw_d_min = self.raw_decisecond_delta(
                            float(min_end), current_spat_deciseconds
                        ) / 10.0
                        d_min = self.normalize_decisecond_delta(
                            float(min_end) - current_spat_deciseconds
                        ) / 10.0
                    else:
                        raw_d_min = None
                    if max_end is not None:
                        raw_d_max = self.raw_decisecond_delta(
                            float(max_end), current_spat_deciseconds
                        ) / 10.0
                        d_max = self.normalize_decisecond_delta(
                            float(max_end) - current_spat_deciseconds
                        ) / 10.0
                    else:
                        raw_d_max = None
                    all_events_description = "; ".join(
                        self.describe_event(event) for event in state.events
                    )
                    self.get_logger().info(
                        "SPaT debug | "
                        f"intersection={intersection_id} "
                        f"signal_group={signal_group} "
                        f"unique_id={configured_signal.unique_signal_id} "
                        f"moy={source_moy} "
                        f"time_stamp_ms={source_timestamp_ms} "
                        f"current_spat_ds={current_spat_deciseconds:.1f} "
                        f"selected=({self.describe_event(active_event)}) "
                        f"raw_d_min_s={raw_d_min} "
                        f"raw_d_max_s={raw_d_max} "
                        f"d_min_s={d_min} "
                        f"d_max_s={d_max} "
                        f"selected_end_ds={offset_deciseconds} "
                        f"remaining_s={remaining_seconds} "
                        f"events=[{all_events_description}]"
                    )

                self.snapshots[configured_signal.unique_signal_id] = SignalSnapshot(
                    event_state=int(active_event.event_state),
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
