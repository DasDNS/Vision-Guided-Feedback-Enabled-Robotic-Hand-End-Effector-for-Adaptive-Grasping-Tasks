#!/usr/bin/env python3
import sys
import time
import threading
import re
from collections import deque
from typing import Optional
from threading import Lock

from PySide6.QtCore import QObject, Signal, QTimer, Qt
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QGroupBox,
    QMessageBox, QComboBox, QScrollArea, QLineEdit,
    QGridLayout, QPlainTextEdit
)

import serial
import serial.tools.list_ports

# ----------------------------
# ROS 2 (Jazzy) imports
# ----------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString

# ----------------------------
# ROS topic names (edit if needed)
# ----------------------------
ROS_TOPIC_REQUEST = "finger_pattern_request"   # UI -> vision laptop
ROS_TOPIC_PATTERN = "finger_pattern"           # vision laptop -> UI
ROS_TOPIC_ACK = "finger_pattern_ack"           # UI -> vision laptop (confirmation)

PATTERN_RE = re.compile(r"^[01]{5}$")
STATE_RE = re.compile(r".*\[STATE\]\s+([A-Z_]+)\s*$")
FSR_RE = re.compile(r"^(?:\d+\s*,\s*)?FSR Live:\s*")
CURRENT_RE = re.compile(r"^\d+\s*,\s*\d+\s*,.*mA.*$")

ALLOWED_CMD_RE = re.compile(r"^(?:[01]{5}|2|3)$")

FINGER_NAMES = ["Pinky", "Ring", "Middle", "Index", "Thumb"]


# ----------------------------
# Port helpers
# ----------------------------
def list_candidate_ports():
    ports = list(serial.tools.list_ports.comports())
    return [p.device for p in ports if ("ttyUSB" in p.device) or ("ttyACM" in p.device)]


def find_stm32_port():
    ports = list_candidate_ports()
    return ports[0] if ports else None


def classify_mcu_line(s: str) -> str:
    if not s:
        return "status"
    if STATE_RE.match(s):
        return "fsm"
    if FSR_RE.match(s):
        return "fsr"
    if CURRENT_RE.match(s) and (",") in s:
        return "current"
    return "status"


# ----------------------------
# Finger selection widget
# ----------------------------
class FingerTableWidget(QWidget):
    def __init__(self, finger_names):
        super().__init__()
        self.finger_names = finger_names

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        self.pattern_label = QLabel("Pattern: (none)")
        self.pattern_label.setAlignment(Qt.AlignCenter)
        self.pattern_label.setObjectName("patternCaption")

        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(6)

        self._name_labels = []
        self._dot_labels = []

        for col, name in enumerate(self.finger_names):
            name_lbl = QLabel(name)
            name_lbl.setAlignment(Qt.AlignCenter)
            name_lbl.setObjectName("fingerName")
            self._name_labels.append(name_lbl)

            dot_lbl = QLabel("●")
            dot_lbl.setAlignment(Qt.AlignCenter)
            dot_lbl.setObjectName("fingerDot")
            self._dot_labels.append(dot_lbl)

            grid.addWidget(name_lbl, 0, col)
            grid.addWidget(dot_lbl, 1, col)

        root.addWidget(self.pattern_label)
        root.addLayout(grid)
        self.set_pattern(None)

    def set_pattern(self, pat: Optional[str]):
        if pat is None or not PATTERN_RE.match(pat):
            self.pattern_label.setText("Pattern: (none)")
            for dot in self._dot_labels:
                dot.setStyleSheet("color: #b0bec5; font-size: 22px;")
            return

        self.pattern_label.setText(f"Pattern: {pat}")
        for i, ch in enumerate(pat):
            if ch == "1":
                self._dot_labels[i].setStyleSheet("color: #2e7d32; font-size: 22px;")
            else:
                self._dot_labels[i].setStyleSheet("color: #c62828; font-size: 22px;")


# ----------------------------
# Serial manager
# ----------------------------
class SerialManager(QObject):
    sig_connected = Signal(str)
    sig_disconnected = Signal()
    sig_error = Signal(str)
    sig_rx_line = Signal(str)
    sig_log_line = Signal(str)

    def __init__(self):
        super().__init__()
        self._ser = None
        self._stop = False
        self._error_latched = False

    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect_port(self, port: str, baud: int = 115200):
        if self.is_connected():
            self.sig_log_line.emit("Serial already connected.")
            return
        try:
            self._ser = serial.Serial(port, baud, timeout=0.2, exclusive=True)
            time.sleep(2.0)
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass

            self._stop = False
            self._error_latched = False
            threading.Thread(target=self._reader_loop, daemon=True).start()

            self.sig_connected.emit(port)
            self.sig_log_line.emit(f"Connected: {port} @ {baud} (exclusive)")
        except Exception as e:
            self._ser = None
            self.sig_error.emit(f"Serial connect failed: {e}")

    def disconnect_port(self):
        self._stop = True
        try:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
        finally:
            self._ser = None

        self.sig_disconnected.emit()
        self.sig_log_line.emit("Disconnected.")

    def write_line_lf(self, text: str):
        """MCU is line-based: always LF \\n."""
        if not self.is_connected():
            self.sig_error.emit("Not connected to serial.")
            return
        try:
            payload = (text + "\n").encode("utf-8", errors="replace")
            self._ser.write(payload)
            self._ser.flush()
            shown = text.replace("\r", "\\r").replace("\n", "\\n")
            self.sig_log_line.emit(f"TX: '{shown}' ending=LF bytes={payload!r}")
        except Exception as e:
            if not self._error_latched:
                self._error_latched = True
                self.sig_error.emit(f"Serial write failed: {e}")
            self.disconnect_port()

    def _reader_loop(self):
        while not self._stop:
            try:
                if not self._ser:
                    break
                raw = self._ser.readline()
                if not raw:
                    continue
                s = raw.decode("utf-8", errors="replace").strip()
                if not s:
                    continue
                self.sig_rx_line.emit(s)
                self.sig_log_line.emit(f"RX: {s}")

            except (serial.SerialException, OSError) as e:
                if not self._error_latched:
                    self._error_latched = True
                    self.sig_error.emit(f"Serial disconnected (device removed?): {e}")
                self.disconnect_port()
                break
            except Exception as e:
                if self._stop:
                    break
                if not self._error_latched:
                    self._error_latched = True
                    self.sig_error.emit(f"Serial read error: {e}")
                self.disconnect_port()
                break


# ----------------------------
# ROS worker (runs rclpy spin in background)
# ----------------------------
class RosPatternNode(Node):
    def __init__(self, outbound_queue: deque, q_lock: Lock, sig_pattern_cb, sig_log_cb, sig_status_cb, sig_event_cb):
        super().__init__("ui_pattern_client")

        self._out_q = outbound_queue
        self._q_lock = q_lock
        self._sig_pattern_cb = sig_pattern_cb
        self._sig_log_cb = sig_log_cb
        self._sig_status_cb = sig_status_cb
        self._sig_event_cb = sig_event_cb  # event hook for UI label

        self.req_pub = self.create_publisher(RosString, ROS_TOPIC_REQUEST, 10)
        self.sub = self.create_subscription(RosString, ROS_TOPIC_PATTERN, self._on_pattern, 10)
        self.ack_pub = self.create_publisher(RosString, ROS_TOPIC_ACK, 10)

        self.timer = self.create_timer(0.05, self._flush_outbound)

        self._sig_log_cb(f"[ROS] Node started. pub={ROS_TOPIC_REQUEST}, sub={ROS_TOPIC_PATTERN}")

        # status tracking + periodic reporting
        self._last_pattern_time = 0.0  # seconds (monotonic time)
        self._status_timer = self.create_timer(0.5, self._publish_status)

    def _flush_outbound(self):
        msg_to_send = None
        with self._q_lock:
            if self._out_q:
                msg_to_send = self._out_q.popleft()

        if msg_to_send is not None:
            m = RosString()
            m.data = msg_to_send

            if msg_to_send.startswith("ACK:"):
                self.ack_pub.publish(m)
                self._sig_log_cb(f"[ROS] Published ACK: {msg_to_send!r}")
                try:
                    self._sig_event_cb("ack_sent", msg_to_send[4:])
                except Exception:
                    pass
            else:
                self.req_pub.publish(m)
                self._sig_log_cb(f"[ROS] Published request: {msg_to_send!r}")
                try:
                    self._sig_event_cb("request_sent", msg_to_send)
                except Exception:
                    pass

    def _on_pattern(self, msg: RosString):
        data = (msg.data or "").strip()
        self._last_pattern_time = time.monotonic()
        self._sig_log_cb(f"[ROS] RX pattern topic: {data!r}")
        try:
            self._sig_event_cb("pattern_received", data)
        except Exception:
            pass
        self._sig_pattern_cb(data)

    def _publish_status(self):
        pubs_on_pattern = 0
        subs_on_request = 0
        try:
            pubs_on_pattern = self.count_publishers(ROS_TOPIC_PATTERN)
        except Exception:
            pubs_on_pattern = -1

        try:
            subs_on_request = self.count_subscribers(ROS_TOPIC_REQUEST)
        except Exception:
            subs_on_request = -1

        now = time.monotonic()
        age = (now - self._last_pattern_time) if self._last_pattern_time > 0 else None

        status = {
            "pubs_on_pattern": pubs_on_pattern,
            "subs_on_request": subs_on_request,
            "last_pattern_age_s": age,
        }
        self._sig_status_cb(status)


class RosWorker(QObject):
    sig_log = Signal(str)
    sig_pattern = Signal(str)
    sig_ready = Signal(bool)
    sig_status = Signal(dict)
    sig_event = Signal(str, str)  # ("request_sent"/"pattern_received"/"ack_sent", payload)

    def __init__(self):
        super().__init__()
        self._thread = None
        self._running = False

        self._out_q = deque()
        self._q_lock = Lock()

        self._executor = None
        self._node = None

    def start(self):
        if self._running:
            return
        self._running = True

        def runner():
            try:
                rclpy.init(args=None)
                self._executor = rclpy.executors.SingleThreadedExecutor()

                self._node = RosPatternNode(
                    outbound_queue=self._out_q,
                    q_lock=self._q_lock,
                    sig_pattern_cb=lambda s: self.sig_pattern.emit(s),
                    sig_log_cb=lambda s: self.sig_log.emit(s),
                    sig_status_cb=lambda d: self.sig_status.emit(d),
                    sig_event_cb=lambda ev, payload: self.sig_event.emit(ev, payload),
                )
                self._executor.add_node(self._node)
                self.sig_ready.emit(True)
                self._executor.spin()
            except Exception as e:
                self.sig_log.emit(f"[ROS] ERROR: {e}")
                self.sig_ready.emit(False)
            finally:
                try:
                    if self._executor and self._node:
                        self._executor.remove_node(self._node)
                        self._node.destroy_node()
                except Exception:
                    pass
                try:
                    if rclpy.ok():
                        rclpy.shutdown()
                except Exception:
                    pass
                self._running = False

        self._thread = threading.Thread(target=runner, daemon=True)
        self._thread.start()

    def stop(self):
        try:
            if self._executor:
                self._executor.shutdown()
        except Exception:
            pass
        self._running = False

    def request_pattern(self):
        with self._q_lock:
            self._out_q.append("REQ")

    def ack_pattern(self, pat: str):
        with self._q_lock:
            self._out_q.append(f"ACK:{pat}")


# ----------------------------
# UI
# ----------------------------
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_mgr = SerialManager()

        # ROS worker (background)
        self.ros = RosWorker()
        self.ros.start()

        # ROS status state vars
        self._ros_local_ok = False
        self._ros_status_dict = {"pubs_on_pattern": 0, "subs_on_request": 0, "last_pattern_age_s": None}

        # transient ROS event text (request/pattern/ack)
        self._ros_event_text = ""
        self._ros_event_color = "#455a64"
        self._ros_event_timer = QTimer(self)
        self._ros_event_timer.setSingleShot(True)
        self._ros_event_timer.timeout.connect(self._clear_ros_event)

        self._latest_pattern: Optional[str] = None
        self._pattern_sent_to_mcu: bool = False

        self._waiting_for_pattern = False

        self.setWindowTitle("Robotic End Effector Control UI (Serial + ROS2 Pattern)")
        self.setMinimumWidth(1050)

        # SAFE FIX: queue *everything*, flush via ONE timer
        self._rx_q = deque(maxlen=5000)
        self._log_q = deque(maxlen=5000)

        self._flush_timer = QTimer(self)
        self._flush_timer.setInterval(50)
        self._flush_timer.timeout.connect(self._flush_queues)
        self._flush_timer.start()

        # Track state for highlight
        self._last_state = None
        self._state_flash_timer = QTimer(self)
        self._state_flash_timer.setSingleShot(True)
        self._state_flash_timer.timeout.connect(self._end_state_flash)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)

        content = QWidget()
        root = QVBoxLayout(content)
        root.setSpacing(12)

        title = QLabel("Robotic End Effector Control Panel")
        title.setObjectName("title")
        root.addWidget(title)

        # ---------------- Connection ----------------
        conn_group = QGroupBox("Connection (MCU Serial)")
        conn_layout = QHBoxLayout()

        self.port_combo = QComboBox()
        self.btn_refresh_ports = QPushButton("Refresh Ports")
        self.btn_auto = QPushButton("Auto-Detect")
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setEnabled(False)

        self.port_status = QLabel("Status: Not connected")
        self.port_status.setObjectName("portStatus")

        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_combo, stretch=1)
        conn_layout.addWidget(self.btn_refresh_ports)
        conn_layout.addWidget(self.btn_auto)
        conn_layout.addWidget(self.btn_connect)
        conn_layout.addWidget(self.btn_disconnect)
        conn_layout.addWidget(self.port_status, stretch=2)
        conn_group.setLayout(conn_layout)
        root.addWidget(conn_group)

        # ---------------- Pattern from ROS ----------------
        ros_group = QGroupBox("Vision Pattern (ROS 2 Jazzy)")
        ros_layout = QHBoxLayout()

        self.btn_request_pattern = QPushButton("Request Pattern (ROS)")
        self.btn_send_pattern_mcu = QPushButton("Send Pattern to MCU")
        self.btn_send_pattern_mcu.setEnabled(False)

        self.ros_status = QLabel("ROS: starting...")
        self.ros_status.setObjectName("rosStatus")
        self.ros_status.setTextFormat(Qt.RichText)  # <-- allow colored 2nd line via HTML

        ros_layout.addWidget(self.btn_request_pattern)
        ros_layout.addWidget(self.btn_send_pattern_mcu)
        ros_layout.addWidget(self.ros_status, stretch=1)
        ros_group.setLayout(ros_layout)
        root.addWidget(ros_group)

        # ---------------- Controls ----------------
        ctrl_group = QGroupBox("Controls (MCU)")
        ctrl_layout = QHBoxLayout()

        self.btn_start = QPushButton("  Start Grasping  ")
        self.btn_reset_open = QPushButton("  Reset/Open  ")
        self.btn_clear = QPushButton("Clear UI")

        ctrl_layout.addWidget(self.btn_start)
        ctrl_layout.addWidget(self.btn_reset_open)
        ctrl_layout.addStretch(1)
        ctrl_layout.addWidget(self.btn_clear)

        ctrl_group.setLayout(ctrl_layout)
        root.addWidget(ctrl_group)

        # ---------------- Optional manual send (still useful for debug) ----------------
        tx_group = QGroupBox("Manual Send (Debug) — Only allowed: 5-bit pattern OR 2 OR 3")
        tx_layout = QHBoxLayout()

        self.tx_input = QLineEdit()
        self.tx_input.setPlaceholderText("Type 00000..11111, or 2, or 3 then Enter")
        self.btn_send = QPushButton("Send")

        tx_layout.addWidget(QLabel("Command:"))
        tx_layout.addWidget(self.tx_input, stretch=2)
        tx_layout.addWidget(self.btn_send)

        tx_group.setLayout(tx_layout)
        root.addWidget(tx_group)

        # ---------------- Active fingers + FSM state ----------------
        info_group = QGroupBox("Grasp Selection + FSM State")
        info_layout = QHBoxLayout()

        self.active_fingers_widget = FingerTableWidget(["Little", "Ring", "Middle", "Index", "Thumb"])

        self.fsm_state_label = QLabel("Waiting for MCU state...")
        self.fsm_state_label.setAlignment(Qt.AlignCenter)
        self.fsm_state_label.setMinimumHeight(90)
        self.fsm_state_label.setObjectName("fsmStateLabel")

        info_layout.addWidget(self._wrap_box("Active Fingers", self.active_fingers_widget), stretch=1)
        info_layout.addWidget(self._wrap_box("FSM State", self.fsm_state_label), stretch=1)
        info_group.setLayout(info_layout)
        root.addWidget(info_group)

        # ---------------- FSR / Current / Status / Debug ----------------
        fsr_group = QGroupBox("FSR Values (FSR Live: ...)")
        fsr_layout = QVBoxLayout()
        self.fsr_box = QPlainTextEdit()
        self.fsr_box.setReadOnly(True)
        self.fsr_box.setUndoRedoEnabled(False)
        self.fsr_box.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.fsr_box.setMinimumHeight(160)
        self.fsr_box.setMaximumBlockCount(100)
        fsr_layout.addWidget(self.fsr_box)
        fsr_group.setLayout(fsr_layout)
        root.addWidget(fsr_group)

        cur_group = QGroupBox("Current Values (millis,pulse, ...)")
        cur_layout = QVBoxLayout()
        self.current_box = QPlainTextEdit()
        self.current_box.setReadOnly(True)
        self.current_box.setUndoRedoEnabled(False)
        self.current_box.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.current_box.setMinimumHeight(160)
        self.current_box.setMaximumBlockCount(100)
        cur_layout.addWidget(self.current_box)
        cur_group.setLayout(cur_layout)
        root.addWidget(cur_group)

        status_group = QGroupBox("MCU Status / Other Messages")
        status_layout = QVBoxLayout()
        self.status_box = QPlainTextEdit()
        self.status_box.setReadOnly(True)
        self.status_box.setUndoRedoEnabled(False)
        self.status_box.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.status_box.setMinimumHeight(180)
        self.status_box.setMaximumBlockCount(150)
        status_layout.addWidget(self.status_box)
        status_group.setLayout(status_layout)
        root.addWidget(status_group)

        log_group = QGroupBox("Debug Log (Serial + ROS)")
        log_layout = QVBoxLayout()
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setUndoRedoEnabled(False)
        self.log_box.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.log_box.setMinimumHeight(160)
        self.log_box.setMaximumBlockCount(200)
        log_layout.addWidget(self.log_box)
        log_group.setLayout(log_layout)
        root.addWidget(log_group)

        scroll.setWidget(content)
        win_layout = QVBoxLayout(self)
        win_layout.addWidget(scroll)

        self.apply_theme()

        # ---------------- signals (serial) ----------------
        self.btn_refresh_ports.clicked.connect(self.refresh_ports)
        self.btn_auto.clicked.connect(self.auto_detect_port)
        self.btn_connect.clicked.connect(self.connect_serial)
        self.btn_disconnect.clicked.connect(self.disconnect_serial)

        self.btn_start.clicked.connect(self.start_grasp_guarded)
        self.btn_reset_open.clicked.connect(lambda: self._send_allowed("3"))
        self.btn_clear.clicked.connect(self.clear_ui)

        self.btn_send.clicked.connect(self.send_from_ui)
        self.tx_input.returnPressed.connect(self.send_from_ui)

        self.serial_mgr.sig_connected.connect(self.on_serial_connected)
        self.serial_mgr.sig_disconnected.connect(self.on_serial_disconnected)
        self.serial_mgr.sig_error.connect(self.on_error)

        self.serial_mgr.sig_rx_line.connect(self._enqueue_rx_line)
        self.serial_mgr.sig_log_line.connect(self._enqueue_log_line)

        # ---------------- signals (ROS) ----------------
        self.btn_request_pattern.clicked.connect(self.request_pattern_ros)
        self.btn_send_pattern_mcu.clicked.connect(self.send_ros_pattern_to_mcu)

        self.ros.sig_log.connect(self._enqueue_log_line)
        self.ros.sig_ready.connect(self.on_ros_ready)
        self.ros.sig_pattern.connect(self.on_ros_pattern_received)
        self.ros.sig_status.connect(self.on_ros_status)
        self.ros.sig_event.connect(self.on_ros_event)

        self.refresh_ports()
        self.active_fingers_widget.set_pattern(None)
        self._set_state_display("Waiting for MCU state...", is_boot=True)

        self._update_ros_status_label()

    def closeEvent(self, event):
        try:
            self.ros.stop()
        except Exception:
            pass
        super().closeEvent(event)

    def _wrap_box(self, title: str, widget: QWidget) -> QWidget:
        box = QGroupBox(title)
        layout = QVBoxLayout()
        layout.addWidget(widget)
        box.setLayout(layout)
        return box

    def _enqueue_log_line(self, s: str):
        self._log_q.append(s)

    def _enqueue_rx_line(self, s: str):
        self._rx_q.append(s)

    def on_ros_ready(self, ok: bool):
        self._ros_local_ok = bool(ok)
        if not ok:
            self._enqueue_log_line("[ROS] Not ready. Did you source ROS 2 Jazzy before running this UI?")
        self._update_ros_status_label()

    def on_ros_status(self, d: dict):
        self._ros_status_dict = d or self._ros_status_dict
        self._update_ros_status_label()

    def on_ros_event(self, ev: str, payload: str):
        ev = (ev or "").strip()
        payload = (payload or "").strip()

        if ev == "request_sent":
            self._ros_event_text = "ROS request sent"
            self._ros_event_color = "#1565c0"  # blue
        elif ev == "pattern_received":
            if PATTERN_RE.match(payload):
                self._ros_event_text = f"ROS pattern received ({payload})"
            else:
                self._ros_event_text = "ROS pattern received"
            self._ros_event_color = "#2e7d32"  # green
        elif ev == "ack_sent":
            self._ros_event_text = "ROS ACK sent"
            self._ros_event_color = "#6a1b9a"  # purple
        else:
            self._ros_event_text = f"ROS event: {ev}"
            self._ros_event_color = "#455a64"  # gray

        self._update_ros_status_label()
        self._ros_event_timer.start(1500)

    def _clear_ros_event(self):
        self._ros_event_text = ""
        self._ros_event_color = "#455a64"
        self._update_ros_status_label()

    def _update_ros_status_label(self):
        if not self._ros_local_ok:
            base_msg = "ROS: not started / init failed"
            base_color = "#c62828"
            self.btn_request_pattern.setEnabled(False)
        else:
            pubs = self._ros_status_dict.get("pubs_on_pattern", 0)
            subs = self._ros_status_dict.get("subs_on_request", 0)
            age = self._ros_status_dict.get("last_pattern_age_s", None)

            have_remote_pattern_pub = (pubs is not None and pubs >= 1)
            have_remote_request_sub = (subs is not None and subs >= 1)

            if have_remote_pattern_pub and have_remote_request_sub:
                if age is None:
                    base_msg = "ROS: connected (no pattern received yet)"
                else:
                    base_msg = f"ROS: connected (last pattern {age:.1f}s ago)"
                base_color = "#2e7d32"
                self.btn_request_pattern.setEnabled(True)

            elif have_remote_request_sub and not have_remote_pattern_pub:
                base_msg = "ROS: request link OK, but no pattern publisher found"
                base_color = "#f57c00"
                self.btn_request_pattern.setEnabled(True)

            elif have_remote_pattern_pub and not have_remote_request_sub:
                base_msg = "ROS: pattern publisher found, but no request subscriber found"
                base_color = "#f57c00"
                self.btn_request_pattern.setEnabled(True)

            else:
                base_msg = "ROS: local OK, but no remote peer detected (check ROS_DOMAIN_ID / network / bringup)"
                base_color = "#c62828"
                self.btn_request_pattern.setEnabled(True)

        # Build rich text: base line in base_color, event line in event_color (no ✅)
        if self._ros_event_text:
            html = (
                f"<div style='font-weight:800; color:{base_color};'>{base_msg}</div>"
                f"<div style='font-weight:800; color:{self._ros_event_color};'>{self._ros_event_text}</div>"
            )
            self.ros_status.setText(html)
        else:
            # keep it simple (one line) — can be plain or rich; we keep rich for consistency
            html = f"<div style='font-weight:800; color:{base_color};'>{base_msg}</div>"
            self.ros_status.setText(html)

    def request_pattern_ros(self):
        self._waiting_for_pattern = True
        self._pattern_sent_to_mcu = False
        self.btn_send_pattern_mcu.setEnabled(False)
        self.ros.request_pattern()
        self._enqueue_log_line("[UI] Requested pattern from vision system (ROS).")

    def on_ros_pattern_received(self, pat: str):
        if not self._waiting_for_pattern:
            self._enqueue_log_line(f"[UI] Ignored ROS pattern (not requested): {pat}")
            return

        pat = (pat or "").strip()
        if not PATTERN_RE.match(pat):
            self._enqueue_log_line(f"[UI] BLOCKED ROS pattern (not 5-bit): {pat!r}")
            return

        self._waiting_for_pattern = False

        self._latest_pattern = pat
        self._pattern_sent_to_mcu = False
        self.btn_send_pattern_mcu.setEnabled(True)

        self.active_fingers_widget.set_pattern(pat)
        self._enqueue_log_line(f"[UI] ROS pattern received: {pat}")

        self.ros.ack_pattern(pat)
        self._enqueue_log_line(f"[UI] Sent ACK for pattern: {pat}")

    def send_ros_pattern_to_mcu(self):
        if self._latest_pattern is None:
            QMessageBox.warning(self, "No pattern", "No ROS pattern received yet.")
            return
        if not self._ensure_connected():
            return

        self.serial_mgr.write_line_lf(self._latest_pattern)
        self._pattern_sent_to_mcu = True
        self._enqueue_log_line(f"[UI] Sent pattern to MCU: {self._latest_pattern}")

    def _state_color(self, st: str) -> str:
        st = (st or "").strip().upper()
        return {
            "IDLE": "#eef7ff",
            "RESETTING": "#fff8e1",
            "CLOSING_FAST": "#e8f5e9",
            "CLOSING_SLOW": "#f1f8e9",
            "TIGHTEN": "#fff3e0",
            "HOLD": "#ede7f6",
            "SETTLE": "#eceff1",
            "RECOVER": "#e3f2fd",
        }.get(st, "#f5f5f5")

    def _set_state_display(self, st: str, is_boot: bool = False):
        safe = (st or "").strip()
        bg = self._state_color(safe)

        self.fsm_state_label.setText(safe if safe else "—")

        if not is_boot and safe and safe != self._last_state:
            self._last_state = safe
            self.fsm_state_label.setStyleSheet(
                f"QLabel {{ background: {bg}; border: 2px solid #90a4ae; border-radius: 10px; padding: 8px; "
                f"font-size: 26px; font-weight: 800; }}"
            )
            self._state_flash_timer.start(350)
        else:
            self.fsm_state_label.setStyleSheet(
                f"QLabel {{ background: {bg}; border: 1px solid #cfd8dc; border-radius: 10px; padding: 8px; "
                f"font-size: 26px; font-weight: 800; }}"
            )

    def _end_state_flash(self):
        current = self._last_state or ""
        bg = self._state_color(current)
        self.fsm_state_label.setStyleSheet(
            f"QLabel {{ background: {bg}; border: 1px solid #cfd8dc; border-radius: 10px; padding: 8px; "
            f"font-size: 26px; font-weight: 800; }}"
        )

    def _flush_queues(self):
        pulled = []
        for _ in range(min(400, len(self._rx_q))):
            pulled.append(self._rx_q.popleft())

        log_pulled = []
        for _ in range(min(400, len(self._log_q))):
            log_pulled.append(self._log_q.popleft())

        if not pulled and not log_pulled:
            return

        fsr_lines = []
        cur_lines = []
        status_lines = []
        new_state = None

        for s in pulled:
            kind = classify_mcu_line(s)
            if kind == "fsr":
                fsr_lines.append(s)
            elif kind == "current":
                cur_lines.append(s)
            elif kind == "fsm":
                m = STATE_RE.match(s)
                if m:
                    new_state = m.group(1)
                status_lines.append(s)
            else:
                status_lines.append(s)

        if fsr_lines:
            self.fsr_box.appendPlainText("\n".join(fsr_lines))
        if cur_lines:
            self.current_box.appendPlainText("\n".join(cur_lines))
        if status_lines:
            self.status_box.appendPlainText("\n".join(status_lines))
        if log_pulled:
            self.log_box.appendPlainText("\n".join(log_pulled))

        if new_state:
            self._set_state_display(new_state)

    def clear_ui(self):
        self.active_fingers_widget.set_pattern(None)
        self._set_state_display("Waiting for MCU state...", is_boot=True)
        self.fsr_box.clear()
        self.current_box.clear()
        self.status_box.clear()
        self.log_box.clear()
        self._latest_pattern = None
        self._pattern_sent_to_mcu = False
        self.btn_send_pattern_mcu.setEnabled(False)
        self._clear_ros_event()

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list_candidate_ports()
        if not ports:
            self.port_combo.addItem("(no ttyUSB/ttyACM found)")
            self.port_combo.setEnabled(False)
        else:
            self.port_combo.setEnabled(True)
            for p in ports:
                self.port_combo.addItem(p)
        self._enqueue_log_line("Ports refreshed.")

    def auto_detect_port(self):
        port = find_stm32_port()
        if not port:
            self.on_error("No serial port detected.")
            return
        idx = self.port_combo.findText(port)
        if idx < 0:
            self.port_combo.addItem(port)
            idx = self.port_combo.findText(port)
        self.port_combo.setCurrentIndex(idx)
        self._enqueue_log_line(f"Auto-detected port: {port}")

    def connect_serial(self):
        if not self.port_combo.isEnabled():
            self.on_error("No valid serial port selected.")
            return
        port = self.port_combo.currentText().strip()
        self.serial_mgr.connect_port(port, 115200)

    def disconnect_serial(self):
        self.serial_mgr.disconnect_port()

    def on_serial_connected(self, port: str):
        self.port_status.setText(f"Status: Connected to {port}")
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self._enqueue_log_line(
            "Connected. Workflow: Request Pattern (ROS) -> Send Pattern to MCU -> Start (2). Reset/Open is (3)."
        )

    def on_serial_disconnected(self):
        self.port_status.setText("Status: Not connected")
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)

    def _ensure_connected(self) -> bool:
        if not self.serial_mgr.is_connected():
            QMessageBox.warning(self, "Not connected", "Connect to STM32 first.")
            return False
        return True

    def _sanitize_cmd(self, raw: str) -> str:
        if raw is None:
            return ""
        raw = raw.replace("\x00", "").strip()
        return "".join(ch for ch in raw if ch in "0123")

    def _send_allowed(self, text: str):
        if not self._ensure_connected():
            return

        cleaned = self._sanitize_cmd(text)
        if cleaned != text.strip():
            self._enqueue_log_line(f"[UI] Cleaned input: {text!r} -> {cleaned!r}")

        if not ALLOWED_CMD_RE.match(cleaned):
            self._enqueue_log_line(f"[UI] BLOCKED: Only allowed: 5-bit pattern OR '2' OR '3'. Got: {cleaned!r}")
            QMessageBox.warning(self, "Blocked", "Only allowed: 00000..11111, or 2, or 3.")
            return

        if PATTERN_RE.match(cleaned):
            self.active_fingers_widget.set_pattern(cleaned)

        if cleaned == "3":
            self.active_fingers_widget.set_pattern(None)

        self.serial_mgr.write_line_lf(cleaned)

    def send_from_ui(self):
        raw = self.tx_input.text()
        if not raw:
            return
        self._send_allowed(raw)
        self.tx_input.clear()

    def start_grasp_guarded(self):
        if not self._ensure_connected():
            return
        if not self._pattern_sent_to_mcu:
            QMessageBox.warning(
                self,
                "Pattern not sent",
                "You must Send Pattern to MCU before starting grasp.\n\nWorkflow:\n1) Request Pattern (ROS)\n2) Send Pattern to MCU\n3) Start Grasping"
            )
            return
        self._send_allowed("2")

    def on_error(self, msg: str):
        self._enqueue_log_line(f"ERROR: {msg}")

        if not self.serial_mgr.is_connected():
            return

        now = time.time()
        if not hasattr(self, "_last_err_popup_t"):
            self._last_err_popup_t = 0.0
            self._last_err_popup_msg = ""

        if msg == self._last_err_popup_msg and (now - self._last_err_popup_t) < 3.0:
            return
        if (now - self._last_err_popup_t) < 1.2:
            return

        self._last_err_popup_t = now
        self._last_err_popup_msg = msg
        QMessageBox.critical(self, "Error", msg)

    def apply_theme(self):
        self.setStyleSheet("""
        QWidget { background-color: #f4f6f8; color: #111111; font-size: 13px; }
        #title { font-size: 20px; font-weight: bold; color: #0b3d91; margin-bottom: 10px; }
        QGroupBox { border: 1px solid #cfd8dc; border-radius: 10px; margin-top: 10px;
                    padding: 10px; background: white; font-weight: bold; }
        QPlainTextEdit, QComboBox, QLineEdit { background: #ffffff; color: #000000; border: 1px solid #cfd8dc;
                                              border-radius: 6px; padding: 6px; }
        QPushButton { background-color: #1976d2; color: white; border-radius: 8px;
                      padding: 10px; font-weight: bold; }
        QPushButton:hover { background-color: #1565c0; }
        QPushButton:pressed { background-color: #0d47a1; }
        QPushButton:disabled { background-color: #90a4ae; color: #f5f5f5; }
        #portStatus { color: #2e7d32; font-weight: 700; }
        QLabel#fingerName { font-weight: 700; color: #263238; }
        QLabel#patternCaption { color: #607d8b; font-weight: 700; }
        """)


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
