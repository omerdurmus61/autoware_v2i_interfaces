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
WINDOW_BG = "#0f172a"
TEXT_PRIMARY = "#e5e7eb"
TEXT_SECONDARY = "#94a3b8"
TEXT_DARK = "#0f172a"
def round_seconds_for_display(seconds: float) -> float:
    """
    Quantize only for visualization at one decimal place without rounding up.
    Keep the internal timing math untouched so the displayed value does not
    jump from 2.8 -> 2.9 or 39.9 -> 40.0 because of sub-decimal jitter.
    """
    non_negative = max(0.0, seconds)
    return int(non_negative * 10.0) / 10.0


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
    update_serial: int


@dataclass
class DisplaySignalState:
    event_state: int
    remaining_seconds: Optional[float]
    end_offset_deciseconds: Optional[int]
    last_source_update_monotonic: float
    last_valid_monotonic: float
    last_processed_serial: int
    pending_event_state: Optional[int] = None
    pending_event_count: int = 0
    pending_end_offset_deciseconds: Optional[int] = None
    pending_end_count: int = 0


class TrafficLightWidget(QtWidgets.QFrame):
    def __init__(self, signal: ConfiguredSignal, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.signal = signal
        self.event_state = MovementEvent.EVENT_UNKNOWN
        self.remaining_seconds: Optional[float] = None
        self._render_signature: Optional[Tuple[int, str, str, str, str]] = None
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
        next_signature = self._build_render_signature(event_state, remaining_seconds)
        if next_signature == self._render_signature:
            self.event_state = event_state
            self.remaining_seconds = remaining_seconds
            return

        self.event_state = event_state
        self.remaining_seconds = remaining_seconds
        self._apply_state()

    def mark_stale(self) -> None:
        next_signature = self._build_render_signature(
            MovementEvent.EVENT_UNKNOWN, None
        )
        if next_signature == self._render_signature:
            self.event_state = MovementEvent.EVENT_UNKNOWN
            self.remaining_seconds = None
            return

        self.event_state = MovementEvent.EVENT_UNKNOWN
        self.remaining_seconds = None
        self._apply_state()

    def _countdown_text(self, active: bool) -> str:
        if not active:
            return "--"
        if self.remaining_seconds is None:
            return "--"
        if self.remaining_seconds <= 0.0:
            return "--"
        return f"{round_seconds_for_display(self.remaining_seconds):.1f}s"

    def _build_render_signature(
        self, event_state: int, remaining_seconds: Optional[float]
    ) -> Tuple[int, str, str, str, str]:
        previous_state = self.event_state
        previous_remaining = self.remaining_seconds
        self.event_state = event_state
        self.remaining_seconds = remaining_seconds

        if self.event_state == MovementEvent.STOP_AND_REMAIN:
            state_name = "Red"
        elif self.event_state == MovementEvent.PROTECTED_CLEARANCE:
            state_name = "Yellow"
        elif self.event_state in (
            MovementEvent.PROTECTED_MOVEMENT_ALLOWED,
            MovementEvent.PERMISSIVE_MOVEMENT_ALLOWED,
        ):
            state_name = "Green"
        else:
            state_name = "Unknown"

        signature = (
            event_state,
            state_name,
            self._countdown_text(self.event_state == MovementEvent.STOP_AND_REMAIN),
            self._countdown_text(self.event_state == MovementEvent.PROTECTED_CLEARANCE),
            self._countdown_text(
                self.event_state
                in (
                    MovementEvent.PROTECTED_MOVEMENT_ALLOWED,
                    MovementEvent.PERMISSIVE_MOVEMENT_ALLOWED,
                )
            ),
        )

        self.event_state = previous_state
        self.remaining_seconds = previous_remaining
        return signature

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
        self._render_signature = (
            self.event_state,
            state_name,
            self.red_lamp.text(),
            self.yellow_lamp.text(),
            self.green_lamp.text(),
        )


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
        self.declare_parameter("display_unknown_hold_sec", 0.75)
        self.declare_parameter("display_state_confirm_count", 1)
        self.declare_parameter("display_end_shift_confirm_count", 2)
        self.declare_parameter("display_small_end_shift_sec", 1.0)
        self.declare_parameter("keep_last_valid_on_unknown", True)
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
        self.display_unknown_hold_sec = float(
            config.get(
                "display_unknown_hold_sec",
                self.get_parameter("display_unknown_hold_sec").value,
            )
        )
        self.display_state_confirm_count = max(
            1,
            int(
                config.get(
                    "display_state_confirm_count",
                    self.get_parameter("display_state_confirm_count").value,
                )
            ),
        )
        self.display_end_shift_confirm_count = max(
            1,
            int(
                config.get(
                    "display_end_shift_confirm_count",
                    self.get_parameter("display_end_shift_confirm_count").value,
                )
            ),
        )
        self.display_small_end_shift_sec = float(
            config.get(
                "display_small_end_shift_sec",
                self.get_parameter("display_small_end_shift_sec").value,
            )
        )
        self.keep_last_valid_on_unknown = bool(
            config.get(
                "keep_last_valid_on_unknown",
                self.get_parameter("keep_last_valid_on_unknown").value,
            )
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
        self.display_states: Dict[int, DisplaySignalState] = {}
        self.snapshot_update_serial = 0

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
        return self.select_event_end_offset_deciseconds(event)

    def select_event_end_offset_deciseconds(self, event: MovementEvent) -> Optional[int]:
        """
        Interpret the network SPaT timing fields as direct relative countdowns
        in deciseconds, then choose the earliest advertised transition.

        Some network feeds populate both min/max fields with small relative
        values instead of absolute time-of-hour offsets. For visualization we
        want the nearest transition bound, regardless of which field carries it.
        """
        candidates: List[int] = []
        if bool(event.timing.has_min_end_time):
            candidates.append(max(0, int(event.timing.min_end_time)))
        if bool(event.timing.has_max_end_time):
            candidates.append(max(0, int(event.timing.max_end_time)))
        if not candidates:
            return None
        return min(candidates)

    def remaining_seconds_from_spat(
        self, event: MovementEvent
    ) -> Optional[float]:
        end_offset_deciseconds = self.select_event_end_offset_deciseconds(event)
        if end_offset_deciseconds is None:
            return None
        return max(0.0, float(end_offset_deciseconds) / 10.0)

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

    def _is_valid_display_state(self, event_state: int) -> bool:
        return self.is_supported_event_state(event_state)

    def _accept_display_snapshot(
        self,
        unique_signal_id: int,
        snapshot: SignalSnapshot,
        event_state: int,
        remaining_seconds: Optional[float],
        end_offset_deciseconds: Optional[int],
        current_monotonic: float,
    ) -> None:
        display_state = self.display_states.get(unique_signal_id)
        if display_state is None:
            self.display_states[unique_signal_id] = DisplaySignalState(
                event_state=event_state,
                remaining_seconds=remaining_seconds,
                end_offset_deciseconds=end_offset_deciseconds,
                last_source_update_monotonic=snapshot.last_update_monotonic,
                last_valid_monotonic=current_monotonic,
                last_processed_serial=snapshot.update_serial,
            )
            return

        display_state.event_state = event_state
        display_state.remaining_seconds = remaining_seconds
        display_state.end_offset_deciseconds = end_offset_deciseconds
        display_state.last_source_update_monotonic = snapshot.last_update_monotonic
        display_state.last_processed_serial = snapshot.update_serial
        display_state.pending_event_state = None
        display_state.pending_event_count = 0
        display_state.pending_end_offset_deciseconds = None
        display_state.pending_end_count = 0
        if self._is_valid_display_state(event_state):
            display_state.last_valid_monotonic = current_monotonic

    def _stabilize_display_state(
        self, unique_signal_id: int, current_monotonic: float
    ) -> Optional[Tuple[int, Optional[float]]]:
        snapshot = self.snapshots.get(unique_signal_id)
        display_state = self.display_states.get(unique_signal_id)

        if snapshot is None:
            if display_state is None:
                return None
            if (
                self.keep_last_valid_on_unknown
                and (current_monotonic - display_state.last_valid_monotonic)
                <= self.display_unknown_hold_sec
            ):
                remaining_seconds = display_state.remaining_seconds
                if remaining_seconds is not None:
                    elapsed = current_monotonic - display_state.last_source_update_monotonic
                    remaining_seconds = max(0.0, remaining_seconds - elapsed)
                return display_state.event_state, remaining_seconds

            self.display_states.pop(unique_signal_id, None)
            return None

        if display_state is None:
            self._accept_display_snapshot(
                unique_signal_id,
                snapshot,
                snapshot.event_state,
                snapshot.remaining_seconds,
                snapshot.end_offset_deciseconds,
                current_monotonic,
            )
            display_state = self.display_states[unique_signal_id]
        elif snapshot.update_serial != display_state.last_processed_serial:
            candidate_state = snapshot.event_state
            candidate_remaining = snapshot.remaining_seconds
            candidate_end = snapshot.end_offset_deciseconds

            if (
                not self._is_valid_display_state(candidate_state)
                and self.keep_last_valid_on_unknown
            ):
                display_state.last_processed_serial = snapshot.update_serial
            elif candidate_state != display_state.event_state:
                displayed_remaining = display_state.remaining_seconds
                if displayed_remaining is not None:
                    displayed_remaining = max(
                        0.0,
                        displayed_remaining
                        - (current_monotonic - display_state.last_source_update_monotonic),
                    )

                if display_state.pending_event_state == candidate_state:
                    display_state.pending_event_count += 1
                else:
                    display_state.pending_event_state = candidate_state
                    display_state.pending_event_count = 1

                if (
                    display_state.pending_event_count >= self.display_state_confirm_count
                    or displayed_remaining == 0.0
                ):
                    self._accept_display_snapshot(
                        unique_signal_id,
                        snapshot,
                        candidate_state,
                        candidate_remaining,
                        candidate_end,
                        current_monotonic,
                    )
                    display_state = self.display_states[unique_signal_id]
                else:
                    display_state.last_processed_serial = snapshot.update_serial
            else:
                display_state.pending_event_state = None
                display_state.pending_event_count = 0

                accept_end_update = False
                if candidate_end is None or display_state.end_offset_deciseconds is None:
                    accept_end_update = True
                else:
                    end_shift_seconds = abs(
                        candidate_end - display_state.end_offset_deciseconds
                    ) / 10.0
                    if end_shift_seconds <= self.display_small_end_shift_sec:
                        accept_end_update = True
                    else:
                        if display_state.pending_end_offset_deciseconds == candidate_end:
                            display_state.pending_end_count += 1
                        else:
                            display_state.pending_end_offset_deciseconds = candidate_end
                            display_state.pending_end_count = 1

                        if (
                            display_state.pending_end_count
                            >= self.display_end_shift_confirm_count
                        ):
                            accept_end_update = True

                if accept_end_update:
                    self._accept_display_snapshot(
                        unique_signal_id,
                        snapshot,
                        candidate_state,
                        candidate_remaining,
                        candidate_end,
                        current_monotonic,
                    )
                    display_state = self.display_states[unique_signal_id]
                else:
                    display_state.last_processed_serial = snapshot.update_serial

        if display_state is None:
            return None

        remaining_seconds = display_state.remaining_seconds
        if remaining_seconds is not None:
            elapsed = current_monotonic - display_state.last_source_update_monotonic
            remaining_seconds = max(0.0, remaining_seconds - elapsed)

        return display_state.event_state, remaining_seconds

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

                offset_deciseconds = self.select_event_end_offset_deciseconds(active_event)
                remaining_seconds = self.remaining_seconds_from_spat(active_event)
                event_state = int(active_event.event_state)

                if (
                    not self.is_supported_event_state(event_state)
                    and self.keep_last_valid_on_unknown
                    and configured_signal.unique_signal_id in self.snapshots
                ):
                    continue

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
                        f"selected=({self.describe_event(active_event)}) "
                        f"selected_end_ds={offset_deciseconds} "
                        f"remaining_s={remaining_seconds} "
                        f"events=[{all_events_description}]"
                    )

                self.snapshot_update_serial += 1
                self.snapshots[configured_signal.unique_signal_id] = SignalSnapshot(
                    event_state=event_state,
                    remaining_seconds=remaining_seconds,
                    last_update_monotonic=receipt_time,
                    source_moy=source_moy,
                    source_timestamp_ms=source_timestamp_ms,
                    end_offset_deciseconds=offset_deciseconds,
                    update_serial=self.snapshot_update_serial,
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
            stabilized = node._stabilize_display_state(
                signal.unique_signal_id, current_monotonic
            )
            if stabilized is None:
                window.mark_stale(signal.unique_signal_id)
                continue

            event_state, remaining_seconds = stabilized
            window.update_signal(signal.unique_signal_id, event_state, remaining_seconds)

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
