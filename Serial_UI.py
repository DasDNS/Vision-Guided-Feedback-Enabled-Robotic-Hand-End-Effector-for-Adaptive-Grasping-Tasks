#!/usr/bin/env python3
import sys
import time
import threading
import re
from collections import deque

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
# Port helpers
# ----------------------------
def list_candidate_ports():
    ports = list(serial.tools.list_ports.comports())
    return [p.device for p in ports if ("ttyUSB" in p.device) or ("ttyACM" in p.device)]


def find_stm32_port():
    ports = list_candidate_ports()
    return ports[0] if ports else None


# ----------------------------
# MCU line classification (your firmware)
# ----------------------------
STATE_RE = re.compile(r".*\[STATE\]\s+([A-Z_]+)\s*$")
FSR_RE = re.compile(r"^(?:\d+\s*,\s*)?FSR Live:\s*")
CURRENT_RE = re.compile(r"^\d+\s*,\s*\d+\s*,.*mA.*$")

PATTERN_RE = re.compile(r"^[01]{5}$")
# Allow ANY 5-bit pattern 00000..11111 OR '2' OR '3'
ALLOWED_CMD_RE = re.compile(r"^(?:[01]{5}|2|3)$")

FINGER_NAMES = ["Pinky", "Ring", "Middle", "Index", "Thumb"]


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
# Finger selection widget (borderless "table" with circles)
# ----------------------------
class FingerTableWidget(QWidget):
    """
    Row 1: finger names (Little/Ring/Middle/Index/Thumb)
    Row 2: colored circles (green selected, red not selected)
    Also shows Pattern: 10101 in a subtle caption.
    """
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

    def set_pattern(self, pat: str | None):
        if pat is None or not PATTERN_RE.match(pat):
            self.pattern_label.setText("Pattern: (none)")
            # default: all "not selected" look but subtle
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
# Serial manager (threaded reader, Qt signals only)
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
        self._error_latched = False  # prevents spam

    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect_port(self, port: str, baud: int = 115200):
        if self.is_connected():
            self.sig_log_line.emit("Serial already connected.")
            return
        try:
            self._ser = serial.Serial(
                port,
                baud,
                timeout=0.2,
                exclusive=True
            )

            # STM32 may reset on open
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
        """Always send a line terminated by LF (firmware is line-based)."""
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
# UI
# ----------------------------
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_mgr = SerialManager()

        self.setWindowTitle("Robotic End Effector Control UI by EGT/21/491 and EGT/21/546")
        self.setMinimumWidth(1000)

        # ---- Queues (ALL UI updates happen only in timer flush) ----
        self._rx_q = deque(maxlen=5000)
        self._log_q = deque(maxlen=5000)

        self._flush_timer = QTimer(self)
        self._flush_timer.setInterval(50)  # 20 fps
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
        conn_group = QGroupBox("Connection (Serial)")
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

        # ---------------- Controls ----------------
        ctrl_group = QGroupBox("Controls")
        ctrl_layout = QHBoxLayout()

        self.btn_start = QPushButton("  Start Grasping  ")      # sends "2"
        self.btn_reset_open = QPushButton("  Reset/Open  ")     # sends "3"
        self.btn_clear = QPushButton("Clear UI")

        ctrl_layout.addWidget(self.btn_start)
        ctrl_layout.addWidget(self.btn_reset_open)
        ctrl_layout.addStretch(1)
        ctrl_layout.addWidget(self.btn_clear)

        ctrl_group.setLayout(ctrl_layout)
        root.addWidget(ctrl_group)

        # ---------------- Serial Monitor style send ----------------
        tx_group = QGroupBox("Send (Only allowed: 5-bit pattern OR 2 OR 3)")
        tx_layout = QHBoxLayout()

        self.tx_input = QLineEdit()
        self.tx_input.setPlaceholderText("Type 00000..11111 (pattern) or 2 (Start) or 3 (Reset/Open), then press Enter")
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

        # ✅ Use QLabel for FSM state (fast, never “stalls”, easy big centered text)
        self.fsm_state_label = QLabel("Waiting for MCU state...")
        self.fsm_state_label.setAlignment(Qt.AlignCenter)
        self.fsm_state_label.setObjectName("fsmStateLabel")
        self.fsm_state_label.setMinimumHeight(90)

        info_layout.addWidget(self._wrap_box("Active Fingers", self.active_fingers_widget), stretch=1)
        info_layout.addWidget(self._wrap_box("FSM State", self.fsm_state_label), stretch=1)
        info_group.setLayout(info_layout)
        root.addWidget(info_group)

        # ---------------- FSR / Current / Status / Debug ----------------
        self.fsr_box = QPlainTextEdit()
        self.current_box = QPlainTextEdit()
        self.status_box = QPlainTextEdit()
        self.log_box = QPlainTextEdit()

        for w in (self.fsr_box, self.current_box, self.status_box, self.log_box):
            w.setReadOnly(True)
            w.setUndoRedoEnabled(False)
            w.setLineWrapMode(QPlainTextEdit.NoWrap)

        # ✅ Keep only recent lines; Qt trims automatically (no cursor hacks)
        self.fsr_box.setMaximumBlockCount(100)
        self.current_box.setMaximumBlockCount(100)
        self.status_box.setMaximumBlockCount(150)
        self.log_box.setMaximumBlockCount(200)

        fsr_group = QGroupBox("FSR Values (FSR Live: ...)")
        fsr_layout = QVBoxLayout()
        self.fsr_box.setMinimumHeight(160)
        fsr_layout.addWidget(self.fsr_box)
        fsr_group.setLayout(fsr_layout)
        root.addWidget(fsr_group)

        cur_group = QGroupBox("Current Values (millis,pulse, ...)")
        cur_layout = QVBoxLayout()
        self.current_box.setMinimumHeight(160)
        cur_layout.addWidget(self.current_box)
        cur_group.setLayout(cur_layout)
        root.addWidget(cur_group)

        status_group = QGroupBox("MCU Status / Other Messages")
        status_layout = QVBoxLayout()
        self.status_box.setMinimumHeight(180)
        status_layout.addWidget(self.status_box)
        status_group.setLayout(status_layout)
        root.addWidget(status_group)

        log_group = QGroupBox("Debug Log")
        log_layout = QVBoxLayout()
        self.log_box.setMinimumHeight(160)
        log_layout.addWidget(self.log_box)
        log_group.setLayout(log_layout)
        root.addWidget(log_group)

        scroll.setWidget(content)
        win_layout = QVBoxLayout(self)
        win_layout.addWidget(scroll)

        self.apply_theme()

        # ---------------- signals ----------------
        self.btn_refresh_ports.clicked.connect(self.refresh_ports)
        self.btn_auto.clicked.connect(self.auto_detect_port)
        self.btn_connect.clicked.connect(self.connect_serial)
        self.btn_disconnect.clicked.connect(self.disconnect_serial)

        self.btn_start.clicked.connect(lambda: self._send_allowed("2"))
        self.btn_reset_open.clicked.connect(lambda: self._send_allowed("3"))
        self.btn_clear.clicked.connect(self.clear_ui)

        self.btn_send.clicked.connect(self.send_from_ui)
        self.tx_input.returnPressed.connect(self.send_from_ui)

        self.serial_mgr.sig_connected.connect(self.on_serial_connected)
        self.serial_mgr.sig_disconnected.connect(self.on_serial_disconnected)
        self.serial_mgr.sig_error.connect(self.on_error)

        # ✅ Only enqueue in signal handlers. Actual UI updates happen in _flush_queues()
        self.serial_mgr.sig_rx_line.connect(self._enqueue_rx_line)
        self.serial_mgr.sig_log_line.connect(self._enqueue_log_line)

        self.refresh_ports()
        self.active_fingers_widget.set_pattern(None)
        self._set_state_display("Waiting for MCU state...", is_boot=True)

    def _wrap_box(self, title: str, widget: QWidget) -> QWidget:
        box = QGroupBox(title)
        layout = QVBoxLayout()
        layout.addWidget(widget)
        box.setLayout(layout)
        return box

    # ---- State display helpers ----
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
        }.get(st, "#f5f5f5")

    def _set_state_display(self, st: str, is_boot: bool = False):
        safe = (st or "").strip()
        self.fsm_state_label.setText(safe if safe else "—")

        bg = self._state_color(safe)
        # Gentle "flash" on change (thicker border briefly)
        if not is_boot and safe and safe != self._last_state:
            self._last_state = safe
            self.fsm_state_label.setStyleSheet(
                f"QLabel {{ background: {bg}; border: 2px solid #90a4ae; border-radius: 10px; "
                f"padding: 8px; font-size: 26px; font-weight: 800; }}"
            )
            self._state_flash_timer.start(350)
        else:
            self.fsm_state_label.setStyleSheet(
                f"QLabel {{ background: {bg}; border: 1px solid #cfd8dc; border-radius: 10px; "
                f"padding: 8px; font-size: 26px; font-weight: 800; }}"
            )

    def _end_state_flash(self):
        current = self._last_state or ""
        bg = self._state_color(current)
        self.fsm_state_label.setStyleSheet(
            f"QLabel {{ background: {bg}; border: 1px solid #cfd8dc; border-radius: 10px; "
            f"padding: 8px; font-size: 26px; font-weight: 800; }}"
        )

    # ---- Queueing (signals) ----
    def _enqueue_rx_line(self, s: str):
        self._rx_q.append(s)

    def _enqueue_log_line(self, s: str):
        self._log_q.append(s)

    # ---- Flush (ONLY place where we touch text widgets) ----
    def _flush_queues(self):
        # Pull some RX
        pulled = []
        for _ in range(min(400, len(self._rx_q))):
            pulled.append(self._rx_q.popleft())

        # Pull some LOG
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

        # Append ONCE per box (fast)
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

    # ---- UI actions ----
    def clear_ui(self):
        self.active_fingers_widget.set_pattern(None)
        self._set_state_display("Waiting for MCU state...", is_boot=True)

        self.fsr_box.clear()
        self.current_box.clear()
        self.status_box.clear()
        self.log_box.clear()

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
        self._enqueue_log_line("Connected. Send pattern (00000..11111) then press Start (2). Reset/Open is (3).")

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
        """
        Remove NUL bytes and other junk that can appear as ^@ on the MCU.
        Keep only 0/1/2/3 characters (since those are the only valid commands here).
        """
        if raw is None:
            return ""
        raw = raw.replace("\x00", "").strip()
        return "".join(ch for ch in raw if ch in "0123")

    def _send_allowed(self, text: str):
        """Send only if it is a 5-bit pattern (00000..11111) or '2' or '3'."""
        if not self._ensure_connected():
            return

        cleaned = self._sanitize_cmd(text)

        if cleaned != text.strip():
            self._enqueue_log_line(f"[UI] Cleaned input: {text!r} -> {cleaned!r}")

        if not ALLOWED_CMD_RE.match(cleaned):
            self._enqueue_log_line(
                f"[UI] BLOCKED: Only allowed commands are: 5-bit pattern (00000..11111) OR '2' OR '3'. Got: {cleaned!r}"
            )
            QMessageBox.warning(
                self,
                "Blocked",
                "Only allowed: 00000..11111 (5-bit pattern), or 2 (Start), or 3 (Reset/Open)."
            )
            return

        # Update finger selection widget if pattern
        if PATTERN_RE.match(cleaned):
            self.active_fingers_widget.set_pattern(cleaned)

        # If Reset/Open, clear finger selection immediately
        if cleaned == "3":
            self.active_fingers_widget.set_pattern(None)

        self.serial_mgr.write_line_lf(cleaned)

    def send_from_ui(self):
        raw = self.tx_input.text()
        if not raw:
            return
        self._send_allowed(raw)
        self.tx_input.clear()

    def on_error(self, msg: str):
        self._enqueue_log_line(f"ERROR: {msg}")

        # If we are already disconnected (common after cable yank), avoid popup spam.
        if not self.serial_mgr.is_connected():
            return

        now = time.time()
        if not hasattr(self, "_last_err_popup_t"):
            self._last_err_popup_t = 0.0
            self._last_err_popup_msg = ""

        # Deduplicate + rate-limit
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

        /* Finger widget subtle typography */
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

