#!/usr/bin/env python3
import sys
import time
import threading
import re
from collections import deque

from PySide6.QtCore import QObject, Signal, QTimer
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QGroupBox, QTextEdit,
    QMessageBox, QComboBox, QScrollArea, QLineEdit
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
# UI append helper (limited lines)
# ----------------------------
def append_limited(text_edit: QTextEdit, line: str, max_lines: int = 400):
    text_edit.append(line)
    doc = text_edit.document()
    if doc.blockCount() > max_lines:
        cursor = text_edit.textCursor()
        cursor.movePosition(cursor.Start)
        extra = doc.blockCount() - max_lines
        for _ in range(extra):
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()


# ----------------------------
# MCU line classification (your firmware)
# ----------------------------
STATE_RE = re.compile(r"^\[STATE\]\s+([A-Z_]+)\s*$")
FSR_RE = re.compile(r"^FSR Live:\s+")
CURRENT_RE = re.compile(r"^\d+,\d+,.+mA.*$")

PATTERN_RE = re.compile(r"^[01]{5}$")
# Allow ANY 5-bit pattern 00000..11111 OR '2' OR '3'
ALLOWED_CMD_RE = re.compile(r"^(?:[01]{5}|2|3)$")

FINGER_NAMES = ["Pinky", "Ring", "Middle", "Index", "Thumb"]


def pattern_to_names(pat: str):
    return [FINGER_NAMES[i] for i, ch in enumerate(pat) if ch == "1"]


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
                self._ser.close()
        except Exception:
            pass
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
            self.sig_error.emit(f"Serial write failed: {e}")

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

            except Exception as e:
                if self._stop:
                    break
                self.sig_error.emit(f"Serial read error: {e}")
                time.sleep(0.2)


# ----------------------------
# UI
# ----------------------------
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_mgr = SerialManager()

        self.setWindowTitle("Robotic End Effector Control UI by EGT/21/491 and EGT/21/546")
        self.setMinimumWidth(1000)

        # ---- RX queue + throttled flush ----
        self._rx_q = deque(maxlen=2000)
        self._flush_timer = QTimer(self)
        self._flush_timer.setInterval(50)  # 20 fps
        self._flush_timer.timeout.connect(self._flush_rx_queue)
        self._flush_timer.start()

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

        self.active_fingers_box = QTextEdit()
        self.active_fingers_box.setReadOnly(True)
        self.active_fingers_box.setMinimumHeight(90)

        self.fsm_state_box = QTextEdit()
        self.fsm_state_box.setReadOnly(True)
        self.fsm_state_box.setMinimumHeight(90)

        info_layout.addWidget(self._wrap_box("Active Fingers", self.active_fingers_box), stretch=1)
        info_layout.addWidget(self._wrap_box("FSM State", self.fsm_state_box), stretch=1)
        info_group.setLayout(info_layout)
        root.addWidget(info_group)

        # ---------------- FSR / Current / Status ----------------
        fsr_group = QGroupBox("FSR Values (FSR Live: ...)")
        fsr_layout = QVBoxLayout()
        self.fsr_box = QTextEdit()
        self.fsr_box.setReadOnly(True)
        self.fsr_box.setMinimumHeight(160)
        fsr_layout.addWidget(self.fsr_box)
        fsr_group.setLayout(fsr_layout)
        root.addWidget(fsr_group)

        cur_group = QGroupBox("Current Values (millis,pulse, ...)")
        cur_layout = QVBoxLayout()
        self.current_box = QTextEdit()
        self.current_box.setReadOnly(True)
        self.current_box.setMinimumHeight(160)
        cur_layout.addWidget(self.current_box)
        cur_group.setLayout(cur_layout)
        root.addWidget(cur_group)

        status_group = QGroupBox("MCU Status / Other Messages")
        status_layout = QVBoxLayout()
        self.status_box = QTextEdit()
        self.status_box.setReadOnly(True)
        self.status_box.setMinimumHeight(180)
        status_layout.addWidget(self.status_box)
        status_group.setLayout(status_layout)
        root.addWidget(status_group)

        log_group = QGroupBox("Debug Log")
        log_layout = QVBoxLayout()
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
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

        self.serial_mgr.sig_rx_line.connect(self._enqueue_rx_line)
        self.serial_mgr.sig_log_line.connect(lambda s: append_limited(self.log_box, s, max_lines=500))

        self.refresh_ports()

        append_limited(self.active_fingers_box, "No pattern yet.", max_lines=50)
        append_limited(self.fsm_state_box, "Waiting for MCU state...", max_lines=50)

    def _wrap_box(self, title: str, widget: QTextEdit) -> QWidget:
        box = QGroupBox(title)
        layout = QVBoxLayout()
        layout.addWidget(widget)
        box.setLayout(layout)
        return box

    # ---- RX handling ----
    def _enqueue_rx_line(self, s: str):
        self._rx_q.append(s)

    def _flush_rx_queue(self):
        if not self._rx_q:
            return

        chunk = []
        for _ in range(min(200, len(self._rx_q))):
            chunk.append(self._rx_q.popleft())

        for s in chunk:
            kind = classify_mcu_line(s)

            if kind == "fsr":
                append_limited(self.fsr_box, s, max_lines=300)
            elif kind == "current":
                append_limited(self.current_box, s, max_lines=300)
            elif kind == "fsm":
                m = STATE_RE.match(s)
                if m:
                    st = m.group(1)
                    self.fsm_state_box.clear()
                    append_limited(self.fsm_state_box, st, max_lines=20)
                append_limited(self.status_box, s, max_lines=300)
            else:
                append_limited(self.status_box, s, max_lines=300)

    # ---- UI actions ----
    def clear_ui(self):
        self.active_fingers_box.clear()
        self.fsm_state_box.clear()
        self.fsr_box.clear()
        self.current_box.clear()
        self.status_box.clear()
        self.log_box.clear()
        append_limited(self.active_fingers_box, "No pattern yet.", max_lines=50)
        append_limited(self.fsm_state_box, "Waiting for MCU state...", max_lines=50)

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
        append_limited(self.log_box, "Ports refreshed.", max_lines=500)

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
        append_limited(self.log_box, f"Auto-detected port: {port}", max_lines=500)

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
        append_limited(
            self.log_box,
            "Connected. Send pattern (00000..11111) then press Start (2). Reset/Open is (3).",
            max_lines=500
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

        # Sanitize before validating
        cleaned = self._sanitize_cmd(text)

        # Helpful debug: show if we removed anything
        if cleaned != text.strip():
            append_limited(self.log_box, f"[UI] Cleaned input: {text!r} -> {cleaned!r}", max_lines=500)

        if not ALLOWED_CMD_RE.match(cleaned):
            append_limited(
                self.log_box,
                f"[UI] BLOCKED: Only allowed commands are: 5-bit pattern (00000..11111) OR '2' OR '3'. Got: {cleaned!r}",
                max_lines=500
            )
            QMessageBox.warning(self, "Blocked", "Only allowed: 00000..11111 (5-bit pattern), or 2 (Start), or 3 (Reset/Open).")
            return

        # Update Active Fingers UI if pattern
        if PATTERN_RE.match(cleaned):
            names = pattern_to_names(cleaned)
            idle = [n for n in FINGER_NAMES if n not in names]
            self.active_fingers_box.clear()
            append_limited(self.active_fingers_box, f"Pattern: {cleaned}", max_lines=50)
            append_limited(self.active_fingers_box, "Enabled: " + (", ".join(names) if names else "(none)"), max_lines=50)
            append_limited(self.active_fingers_box, "Idle: " + (", ".join(idle) if idle else "(none)"), max_lines=50)

        # Firmware is line-based -> always LF
        self.serial_mgr.write_line_lf(cleaned)

    def send_from_ui(self):
        raw = self.tx_input.text()
        if not raw:
            return
        self._send_allowed(raw)
        self.tx_input.clear()

    def on_error(self, msg: str):
        append_limited(self.log_box, f"ERROR: {msg}", max_lines=500)
        QMessageBox.critical(self, "Error", msg)

    def apply_theme(self):
        self.setStyleSheet("""
        QWidget { background-color: #f4f6f8; color: #111111; font-size: 13px; }
        #title { font-size: 20px; font-weight: bold; color: #0b3d91; margin-bottom: 10px; }
        QGroupBox { border: 1px solid #cfd8dc; border-radius: 10px; margin-top: 10px;
                    padding: 10px; background: white; font-weight: bold; }
        QTextEdit, QComboBox, QLineEdit { background: #ffffff; color: #000000; border: 1px solid #cfd8dc;
                                          border-radius: 6px; padding: 6px; }
        QPushButton { background-color: #1976d2; color: white; border-radius: 8px;
                      padding: 10px; font-weight: bold; }
        QPushButton:hover { background-color: #1565c0; }
        QPushButton:pressed { background-color: #0d47a1; }
        QPushButton:disabled { background-color: #90a4ae; color: #f5f5f5; }
        #portStatus { color: #2e7d32; font-weight: 700; }
        """)


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

