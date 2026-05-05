#!/usr/bin/env python3
import sys
import time
import threading
import re

from PySide6.QtCore import QObject, Signal
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
# Keep UI fast (avoid infinite text growth)
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
            cursor.deleteChar()  # remove newline


# ----------------------------
# Classify MCU output for YOUR C++ code
# ----------------------------
CURRENT_RE = re.compile(r"^\d+,\d+,S0=.*mA, S1=.*mA, S2=.*mA, S3=.*mA, S4=.*mA$")

def classify_mcu_line(s: str) -> str:
    if not s:
        return "status"
    if s.startswith("FSR Live:"):
        return "fsr"
    if CURRENT_RE.match(s):
        return "current"
    return "status"


# ----------------------------
# Serial manager
# ----------------------------
class SerialManager(QObject):
    sig_connected = Signal(str)
    sig_disconnected = Signal()
    sig_error = Signal(str)

    sig_status_line = Signal(str)
    sig_fsr_line = Signal(str)
    sig_current_line = Signal(str)
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

            # STM32/adapter sometimes toggles DTR/RTS and resets board when opening.
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

    def write_text(self, text: str, line_ending: str = "NONE"):
        """
        Sends exactly what you type, optionally with a selected line ending.
        line_ending: "NONE", "LF", "CR", "CRLF"
        """
        if not self.is_connected():
            self.sig_error.emit("Not connected to serial.")
            return

        try:
            cmd = text

            if line_ending == "LF":
                cmd += "\n"
            elif line_ending == "CR":
                cmd += "\r"
            elif line_ending == "CRLF":
                cmd += "\r\n"

            payload = cmd.encode("utf-8", errors="replace")
            self._ser.write(payload)
            self._ser.flush()

            shown = text.replace("\r", "\\r").replace("\n", "\\n")
            self.sig_log_line.emit(f"TX: '{shown}' ending={line_ending} bytes={payload!r}")
        except Exception as e:
            self.sig_error.emit(f"Serial write failed: {e}")

    def write_single_char_cmd(self, ch: str):
        """
        Firmware uses Serial.read() and switch(cmd) with '0' and '1'.
        Safest: send exactly ONE byte and NO line ending.
        """
        if not ch or len(ch) != 1:
            self.sig_error.emit("Internal error: command must be one character.")
            return
        self.write_text(ch, line_ending="NONE")

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

                kind = classify_mcu_line(s)
                if kind == "fsr":
                    self.sig_fsr_line.emit(s)
                elif kind == "current":
                    self.sig_current_line.emit(s)
                else:
                    self.sig_status_line.emit(s)

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

        self.setWindowTitle("STM32 Hand Control UI (0/1 Ramp, 10s)")
        self.setMinimumWidth(950)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)

        content = QWidget()
        root = QVBoxLayout(content)
        root.setSpacing(12)

        title = QLabel("STM32 Hand Control Panel")
        title.setObjectName("title")
        root.addWidget(title)

        # Connection
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

        # Quick controls -> 0..1
        quick_group = QGroupBox("Quick Controls (send single-char commands)")
        quick_layout = QHBoxLayout()

        # REQUIRED mapping:
        # 1: 2400 -> 500 (close)
        # 0: 500 -> 2400 (open)
        self.btn_cmd1 = QPushButton("1  Ramp CLOSE → 500 (10s)")
        self.btn_cmd0 = QPushButton("0  Ramp OPEN  → 2400 (10s)")
        self.btn_clear = QPushButton("Clear UI")

        for b in (self.btn_cmd1, self.btn_cmd0, self.btn_clear):
            quick_layout.addWidget(b)

        quick_group.setLayout(quick_layout)
        root.addWidget(quick_group)

        # Serial Monitor style send box
        tx_group = QGroupBox("Send Command (Serial Monitor style)")
        tx_layout = QHBoxLayout()

        self.tx_input = QLineEdit()
        self.tx_input.setPlaceholderText("Type 0 or 1 and press Enter (firmware reads 1 char at a time)")

        self.tx_line_ending = QComboBox()
        self.tx_line_ending.addItems(["NONE", "LF", "CR", "CRLF"])
        self.tx_line_ending.setCurrentText("NONE")

        self.btn_send = QPushButton("Send")

        tx_layout.addWidget(QLabel("Command:"))
        tx_layout.addWidget(self.tx_input, stretch=2)
        tx_layout.addWidget(QLabel("Line ending:"))
        tx_layout.addWidget(self.tx_line_ending)
        tx_layout.addWidget(self.btn_send)

        tx_group.setLayout(tx_layout)
        root.addWidget(tx_group)

        # FSR box
        fsr_group = QGroupBox("FSR Values (lines starting with 'FSR Live:')")
        fsr_layout = QVBoxLayout()
        self.fsr_box = QTextEdit()
        self.fsr_box.setReadOnly(True)
        self.fsr_box.setMinimumHeight(150)
        fsr_layout.addWidget(self.fsr_box)
        fsr_group.setLayout(fsr_layout)
        root.addWidget(fsr_group)

        # Current box
        cur_group = QGroupBox("Current Values (millis,pulse,S0..S4)")
        cur_layout = QVBoxLayout()
        self.current_box = QTextEdit()
        self.current_box.setReadOnly(True)
        self.current_box.setMinimumHeight(150)
        cur_layout.addWidget(self.current_box)
        cur_group.setLayout(cur_layout)
        root.addWidget(cur_group)

        # Status box
        status_group = QGroupBox("Current State / Other Messages")
        status_layout = QVBoxLayout()
        self.status_box = QTextEdit()
        self.status_box.setReadOnly(True)
        self.status_box.setMinimumHeight(180)
        status_layout.addWidget(self.status_box)
        status_group.setLayout(status_layout)
        root.addWidget(status_group)

        # Debug log
        log_group = QGroupBox("Debug Log (TX/RX/connection)")
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

        # UI signals
        self.btn_refresh_ports.clicked.connect(self.refresh_ports)
        self.btn_auto.clicked.connect(self.auto_detect_port)
        self.btn_connect.clicked.connect(self.connect_serial)
        self.btn_disconnect.clicked.connect(self.disconnect_serial)

        # Quick buttons -> send single char exactly
        self.btn_cmd1.clicked.connect(lambda: self.send_single("1"))
        self.btn_cmd0.clicked.connect(lambda: self.send_single("0"))

        self.btn_clear.clicked.connect(self.clear_ui)

        # Send box
        self.btn_send.clicked.connect(self.send_from_ui)
        self.tx_input.returnPressed.connect(self.send_from_ui)

        # Serial signals
        self.serial_mgr.sig_connected.connect(self.on_serial_connected)
        self.serial_mgr.sig_disconnected.connect(self.on_serial_disconnected)
        self.serial_mgr.sig_error.connect(self.on_error)

        self.serial_mgr.sig_fsr_line.connect(lambda s: append_limited(self.fsr_box, s, max_lines=300))
        self.serial_mgr.sig_current_line.connect(lambda s: append_limited(self.current_box, s, max_lines=300))
        self.serial_mgr.sig_status_line.connect(lambda s: append_limited(self.status_box, s, max_lines=300))
        self.serial_mgr.sig_log_line.connect(lambda s: append_limited(self.log_box, s, max_lines=450))

        self.refresh_ports()

    def clear_ui(self):
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
        append_limited(self.log_box, "Ports refreshed.", max_lines=450)

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
        append_limited(self.log_box, f"Auto-detected port: {port}", max_lines=450)

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
        append_limited(self.log_box, "Connected. Use buttons 1 (close) and 0 (open).", max_lines=450)

    def on_serial_disconnected(self):
        self.port_status.setText("Status: Not connected")
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)

    def send_single(self, ch: str):
        if not self.serial_mgr.is_connected():
            QMessageBox.warning(self, "Not connected", "Connect to STM32 first.")
            return
        self.serial_mgr.write_single_char_cmd(ch)

    def send_from_ui(self):
        if not self.serial_mgr.is_connected():
            QMessageBox.warning(self, "Not connected", "Connect to STM32 first.")
            return

        text = self.tx_input.text()
        if not text:
            return

        ending = self.tx_line_ending.currentText()
        self.serial_mgr.write_text(text, line_ending=ending)
        self.tx_input.clear()

        if len(text) > 1 and ending == "NONE":
            append_limited(
                self.log_box,
                "NOTE: Firmware reads 1 char at a time. Multi-char send becomes multiple commands.",
                max_lines=450
            )

    def on_error(self, msg: str):
        append_limited(self.log_box, f"ERROR: {msg}", max_lines=450)
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

