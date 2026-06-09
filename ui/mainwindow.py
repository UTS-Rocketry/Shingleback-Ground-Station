import sys
import random
from PyQt5 import QtWidgets, QtCore
from collections import deque
import pyqtgraph as pg

from core.parser import (
    parse_telemetry,
    parse_continuity,
    build_command,
    CMD_ARM,
    CMD_FIRE,
    CMD_DISARM,
)
from core.lora_worker import LoRaWorker


class MainWindow(QtWidgets.QMainWindow):
    COMMAND_REPEATS = 5
    ARM_CONFIRM_TIMEOUT_MS = 15000
    STATE_IDLE = 0
    STATE_ARMED = 1

    FLIGHT_STATES = {
        0: ("IDLE", "grey", "white"),
        1: ("ARMED", "orange", "black"),
        2: ("POWERED ASCENT", "yellow", "black"),
        3: ("COASTING", "cyan", "black"),
        4: ("APOGEE", "magenta", "white"),
        5: ("DESCENT", "deepskyblue", "black"),
        6: ("LANDED", "lime", "black"),
        7: ("FAULT", "red", "white"),
        8: ("BENCHTEST", "white", "black"),
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Shingleback Ground Station")
        self.resize(1400, 800)

        # Arm state
        self.arm_code = str(random.randint(1000, 9999))
        self.armed = False
        self.pending_arm = False
        self.pending_command = None
        self.flight_state = 0

        n = 100
        self.alt  = deque([0]*n, maxlen=n)
        self.gy_x = deque([0]*n, maxlen=n)
        self.gy_y = deque([0]*n, maxlen=n)
        self.gy_z = deque([0]*n, maxlen=n)
        self.xl_x = deque([0]*n, maxlen=n)
        self.xl_y = deque([0]*n, maxlen=n)
        self.xl_z = deque([0]*n, maxlen=n)
        self.vel  = deque([0]*n, maxlen=n)

        # --- Plots ---
        self.plot_widget = pg.GraphicsLayoutWidget()

        alt_plot = self.plot_widget.addPlot(title="Altitude (m)")
        self.alt_curve = alt_plot.plot(pen='y')
        self.plot_widget.nextRow()

        gy_plot = self.plot_widget.addPlot(title="Gyro (dps)")
        self.gy_x_curve = gy_plot.plot(pen='r', name="X")
        self.gy_y_curve = gy_plot.plot(pen='g', name="Y")
        self.gy_z_curve = gy_plot.plot(pen='b', name="Z")
        self.plot_widget.nextRow()

        xl_plot = self.plot_widget.addPlot(title="IMU Accel (mg)")
        self.xl_x_curve = xl_plot.plot(pen='r', name="X")
        self.xl_y_curve = xl_plot.plot(pen='g', name="Y")
        self.xl_z_curve = xl_plot.plot(pen='b', name="Z")
        self.plot_widget.nextRow()

        vel_plot = self.plot_widget.addPlot(title="Velocity (m/s)")
        self.vel_curve = vel_plot.plot(pen='c')

        # --- Dashboard ---
        dashboard = QtWidgets.QWidget()
        dash_layout = QtWidgets.QVBoxLayout()
        dashboard.setLayout(dash_layout)
        dashboard.setFixedWidth(220)

        def make_label(title):
            lbl = QtWidgets.QLabel(title)
            lbl.setStyleSheet("font-weight: bold; font-size: 13px; margin-top: 8px;")
            return lbl

        def make_value():
            lbl = QtWidgets.QLabel("--")
            lbl.setStyleSheet("font-family: Courier; font-size: 13px;")
            return lbl

        def make_cont_btn(label):
            lbl = QtWidgets.QLabel(f"{label}: --")
            lbl.setStyleSheet("background-color: grey; color: white; font-weight: bold; padding: 4px;")
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            return lbl
    
        # Flight state
        dash_layout.addWidget(make_label("Flight State"))
        self.flight_state_val = QtWidgets.QLabel()
        self.flight_state_val.setAlignment(QtCore.Qt.AlignCenter)
        self.flight_state_val.setWordWrap(True)
        self.flight_state_val.setMinimumHeight(72)
        dash_layout.addWidget(self.flight_state_val)
        self.update_flight_state_indicator()

        # Altitude
        dash_layout.addWidget(make_label("Altitude"))
        self.alt_val = make_value()
        dash_layout.addWidget(self.alt_val)

        # Velocity
        dash_layout.addWidget(make_label("Velocity (m/s)"))
        self.vel_val = make_value()
        dash_layout.addWidget(self.vel_val)

        # IMU Accel
        dash_layout.addWidget(make_label("IMU Accel (mg)"))
        self.xl_x_val = make_value()
        self.xl_y_val = make_value()
        self.xl_z_val = make_value()
        dash_layout.addWidget(self.xl_x_val)
        dash_layout.addWidget(self.xl_y_val)
        dash_layout.addWidget(self.xl_z_val)

        # Gyro
        dash_layout.addWidget(make_label("Gyro (dps)"))
        self.gy_x_val = make_value()
        self.gy_y_val = make_value()
        self.gy_z_val = make_value()
        dash_layout.addWidget(self.gy_x_val)
        dash_layout.addWidget(self.gy_y_val)
        dash_layout.addWidget(self.gy_z_val)

        # H3LIS
        dash_layout.addWidget(make_label("H3LIS (mg)"))
        self.hx_val = make_value()
        self.hy_val = make_value()
        self.hz_val = make_value()
        dash_layout.addWidget(self.hx_val)
        dash_layout.addWidget(self.hy_val)
        dash_layout.addWidget(self.hz_val)

        # Continuity
        dash_layout.addWidget(make_label("Continuity"))
        self.cont_main   = make_cont_btn("Main")
        self.cont_drogue = make_cont_btn("Drogue")
        dash_layout.addWidget(self.cont_main)
        dash_layout.addWidget(self.cont_drogue)

        # Arm code
        dash_layout.addWidget(make_label("Arm Code"))
        self.arm_code_display = QtWidgets.QLabel(self.arm_code)
        self.arm_code_display.setStyleSheet(
            "font-family: Courier; font-size: 22px; font-weight: bold; color: red;"
        )
        dash_layout.addWidget(self.arm_code_display)

        self.arm_input = QtWidgets.QLineEdit()
        self.arm_input.setPlaceholderText("Enter code to arm")
        self.arm_input.setMaxLength(4)
        dash_layout.addWidget(self.arm_input)

        self.arm_btn = QtWidgets.QPushButton("ARM")
        self.arm_btn.setStyleSheet("background-color: orange; font-weight: bold;")
        self.arm_btn.clicked.connect(self.try_arm)
        dash_layout.addWidget(self.arm_btn)

        self.disarm_btn = QtWidgets.QPushButton("DISARM")
        self.disarm_btn.setStyleSheet("background-color: #555; color: white; font-weight: bold;")
        self.disarm_btn.setEnabled(False)
        self.disarm_btn.clicked.connect(lambda: self.do_disarm(send_remote=True))
        dash_layout.addWidget(self.disarm_btn)

        # Commands
        dash_layout.addWidget(make_label("Commands"))
        self.btn_fire_drogue = QtWidgets.QPushButton("Fire Drogue")
        self.btn_fire_main   = QtWidgets.QPushButton("Fire Main")
        self.btn_fire_drogue.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")
        self.btn_fire_main.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")
        self.btn_fire_drogue.setEnabled(False)
        self.btn_fire_main.setEnabled(False)
        self.btn_fire_drogue.clicked.connect(lambda: self.send_command(CMD_FIRE, 1))
        self.btn_fire_main.clicked.connect(lambda: self.send_command(CMD_FIRE, 2))
        dash_layout.addWidget(self.btn_fire_drogue)
        dash_layout.addWidget(self.btn_fire_main)

        dash_layout.addStretch()

        # --- Terminal ---
        self.terminal = QtWidgets.QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setFontFamily("Courier")

        # --- Layout ---
        v_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        v_splitter.addWidget(self.plot_widget)
        v_splitter.addWidget(self.terminal)
        v_splitter.setSizes([550, 150])

        h_layout = QtWidgets.QHBoxLayout()
        h_layout.addWidget(v_splitter)
        h_layout.addWidget(dashboard)

        central = QtWidgets.QWidget()
        central.setLayout(h_layout)
        self.setCentralWidget(central)

        # Worker
        self.worker = LoRaWorker()
        self.worker.data_received.connect(self.on_lora_data)
        self.worker.error_occurred.connect(lambda e: self.terminal.append(f"[ERROR] {e}"))

        self.arm_timeout_timer = QtCore.QTimer(self)
        self.arm_timeout_timer.setSingleShot(True)
        self.arm_timeout_timer.timeout.connect(self.handle_arm_timeout)
        self.worker.start()

    def try_arm(self):
        if self.arm_input.text() == self.arm_code:
            self.begin_arm_request()
        else:
            self.arm_input.clear()
            self.arm_input.setPlaceholderText("Wrong code!")
            self.arm_input.setStyleSheet("border: 2px solid red;")
            self.terminal.append("[SYS] ARM FAILED - wrong code")
            self.terminal.ensureCursorVisible()

    def begin_arm_request(self):
        self.pending_arm = True
        self.armed = False
        self.btn_fire_drogue.setEnabled(False)
        self.btn_fire_main.setEnabled(False)
        self.disarm_btn.setEnabled(True)
        self.arm_btn.setEnabled(False)
        self.arm_input.setEnabled(False)
        self.arm_code_display.setText("ARMING...")
        self.arm_code_display.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: orange;"
        )
        self.terminal.append("[SYS] ARM REQUEST QUEUED - waiting for next telemetry")
        self.terminal.ensureCursorVisible()

        self.send_arm_request()
        self.arm_timeout_timer.start(self.ARM_CONFIRM_TIMEOUT_MS)

        if self.flight_state == self.STATE_ARMED:
            self.confirm_arm()

    def send_arm_request(self):
        if not self.pending_arm:
            return
        if self.flight_state == self.STATE_ARMED:
            self.confirm_arm()
            return
        self.queue_command(
            CMD_ARM,
            0,
            "ARM",
            should_send=lambda: self.pending_arm and self.flight_state != self.STATE_ARMED,
        )

    def confirm_arm(self):
        if not self.pending_arm:
            return

        self.pending_arm = False
        self.clear_pending_command("ARM")
        self.arm_timeout_timer.stop()
        self.armed = True
        self.btn_fire_drogue.setEnabled(True)
        self.btn_fire_main.setEnabled(True)
        self.disarm_btn.setEnabled(True)
        self.arm_btn.setEnabled(False)
        self.arm_input.setEnabled(False)
        self.arm_code_display.setText("ARMED")
        self.arm_code_display.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: lime;"
        )
        self.terminal.append("[SYS] ARM CONFIRMED - flight computer reported ARMED")
        self.terminal.ensureCursorVisible()

    def handle_arm_timeout(self):
        if not self.pending_arm:
            return

        self.pending_arm = False
        self.clear_pending_command("ARM")
        self.armed = False
        self.btn_fire_drogue.setEnabled(False)
        self.btn_fire_main.setEnabled(False)
        self.disarm_btn.setEnabled(False)
        self.arm_btn.setEnabled(True)
        self.arm_input.setEnabled(True)
        self.arm_input.clear()
        self.arm_input.setPlaceholderText("Enter code to arm")
        self.arm_input.setStyleSheet("")
        self.arm_code_display.setText(self.arm_code)
        self.arm_code_display.setStyleSheet(
            "font-family: Courier; font-size: 22px; font-weight: bold; color: red;"
        )
        self.terminal.append("[SYS] ARM TIMEOUT - ARMED telemetry not received")
        self.terminal.ensureCursorVisible()

    def air_packet_length(self, packet: bytes) -> int:
        return len(packet) + LoRaWorker.RADIOHEAD_HEADER_LENGTH

    def clear_pending_command(self, label: str | None = None):
        if self.pending_command is None:
            return
        if label is None or self.pending_command["label"] == label:
            self.pending_command = None

    def queue_command(
        self,
        cmd_id: int,
        channel: int,
        label: str,
        repeats: int | None = None,
        should_send=None,
    ):
        if repeats is None:
            repeats = self.COMMAND_REPEATS

        pkt = build_command(cmd_id, channel)
        self.pending_command = {
            "packet": pkt,
            "label": label,
            "sent": 0,
            "repeats": repeats,
            "should_send": should_send,
        }
        self.terminal.append(
            f"[CMD QUEUED] {label} ({repeats} telemetry-synced packets, payload={len(pkt)}B, air={self.air_packet_length(pkt)}B)"
        )
        self.terminal.ensureCursorVisible()

    def send_pending_command_after_telemetry(self):
        if self.pending_command is None:
            return

        command = self.pending_command
        should_send = command["should_send"]
        if should_send is not None and not should_send():
            self.pending_command = None
            return

        self.worker.send(command["packet"])
        command["sent"] += 1
        self.terminal.append(
            f"[CMD TX] {command['label']} {command['sent']}/{command['repeats']}"
        )
        self.terminal.ensureCursorVisible()

        if command["sent"] >= command["repeats"]:
            self.pending_command = None

    def update_flight_state_indicator(self):
        name, bg_color, fg_color = self.FLIGHT_STATES.get(
            self.flight_state,
            (f"UNKNOWN {self.flight_state}", "red", "white"),
        )
        self.flight_state_val.setText(name)
        self.flight_state_val.setStyleSheet(
            "font-size: 22px; font-weight: bold; padding: 12px 6px; "
            f"background-color: {bg_color}; color: {fg_color}; border-radius: 4px;"
        )

    def do_disarm(self, *, send_remote: bool = True):
        self.pending_arm = False
        self.clear_pending_command("ARM")
        self.arm_timeout_timer.stop()

        if send_remote:
            self.queue_command(CMD_DISARM, 0, "DISARM")

        self.armed = False
        self.btn_fire_drogue.setEnabled(False)
        self.btn_fire_main.setEnabled(False)
        self.disarm_btn.setEnabled(False)
        self.arm_btn.setEnabled(True)
        self.arm_input.setEnabled(True)
        self.arm_input.clear()
        self.arm_input.setPlaceholderText("Enter code to arm")
        self.arm_input.setStyleSheet("")
        self.arm_code = str(random.randint(1000, 9999))
        self.arm_code_display.setText(self.arm_code)
        self.arm_code_display.setStyleSheet(
            "font-family: Courier; font-size: 22px; font-weight: bold; color: red;"
        )
        self.terminal.append("[SYS] DISARMED")
        self.terminal.ensureCursorVisible()

    def send_command(self, cmd_id: int, channel: int):
        if not self.armed:
            self.terminal.append("[CMD BLOCKED] System is disarmed; command not sent")
            self.terminal.ensureCursorVisible()
            return

        if self.flight_state != self.STATE_ARMED:
            state_name = self.FLIGHT_STATES.get(
                self.flight_state,
                (f"UNKNOWN {self.flight_state}", "", ""),
            )[0]
            self.terminal.append(f"[CMD BLOCKED] Flight state {state_name}; expected ARMED")
            self.terminal.ensureCursorVisible()
            return

        label = {1: "Drogue", 2: "Main"}.get(channel, f"ch{channel}")
        self.queue_command(
            cmd_id,
            channel,
            f"FIRE {label}",
            should_send=lambda: self.flight_state == self.STATE_ARMED,
        )
        self.do_disarm(send_remote=False)

    def on_lora_data(self, raw: bytes):
        if len(raw) < 2:
            self.terminal.append(f"[BAD PACKET] too short: {raw.hex()}")
            return
        
        self.terminal.append(f"[RAW] type={raw[1]:#04x} len={len(raw)}") 

        pkt_type = raw[1]

        if pkt_type == 0x01:
            self._handle_telemetry(raw)
        elif pkt_type == 0x02:
            self._handle_continuity(raw)
        else:
            self.terminal.append(f"[UNKNOWN PKT] type={pkt_type:#04x} {raw.hex()}")
        self.terminal.ensureCursorVisible()

    def _handle_telemetry(self, raw: bytes):
        parsed = parse_telemetry(raw)
        if parsed is None:
            self.terminal.append(f"[BAD TELEM] {raw.hex()}")
            return

        self.flight_state = parsed['flight_state']
        self.update_flight_state_indicator()
        if self.pending_arm and self.flight_state == self.STATE_ARMED:
            self.confirm_arm()
        if self.armed:
            fire_enabled = self.flight_state == self.STATE_ARMED
            self.btn_fire_drogue.setEnabled(fire_enabled)
            self.btn_fire_main.setEnabled(fire_enabled)
        self.send_pending_command_after_telemetry()

        self.terminal.append(
            f"[{parsed['sequence']:03d}] "
            f"Alt: {parsed['altitude']:.2f}m | "
            f"Vel: {parsed['velocity']:.2f}m/s | "
            f"P: {parsed['pressure']:.1f}Pa | "
            f"T: {parsed['temperature']:.1f}C | "
            f"H3LIS X:{parsed['h3lis']['x']:.1f} Y:{parsed['h3lis']['y']:.1f} Z:{parsed['h3lis']['z']:.1f} | "
            f"IMU X:{parsed['imu_accel']['x']:.1f} Y:{parsed['imu_accel']['y']:.1f} Z:{parsed['imu_accel']['z']:.1f} | "
            f"Gyro X:{parsed['imu_gyro']['x']:.1f} Y:{parsed['imu_gyro']['y']:.1f} Z:{parsed['imu_gyro']['z']:.1f} | "
            f"State: {parsed['flight_state']}"
        )

        alt  = parsed['altitude']
        vel  = parsed['velocity']
        xl_x, xl_y, xl_z = parsed['imu_accel']['x'], parsed['imu_accel']['y'], parsed['imu_accel']['z']
        gy_x, gy_y, gy_z = parsed['imu_gyro']['x'],  parsed['imu_gyro']['y'],  parsed['imu_gyro']['z']
        hx,   hy,   hz   = parsed['h3lis']['x'],     parsed['h3lis']['y'],     parsed['h3lis']['z']

        self.alt.append(alt);  self.vel.append(vel)
        self.xl_x.append(xl_x); self.xl_y.append(xl_y); self.xl_z.append(xl_z)
        self.gy_x.append(gy_x); self.gy_y.append(gy_y); self.gy_z.append(gy_z)

        self.alt_curve.setData(list(self.alt))
        self.vel_curve.setData(list(self.vel))
        self.gy_x_curve.setData(list(self.gy_x))
        self.gy_y_curve.setData(list(self.gy_y))
        self.gy_z_curve.setData(list(self.gy_z))
        self.xl_x_curve.setData(list(self.xl_x))
        self.xl_y_curve.setData(list(self.xl_y))
        self.xl_z_curve.setData(list(self.xl_z))

        self.alt_val.setText(f"Alt: {alt:.2f} m")
        self.vel_val.setText(f"{vel:.2f} m/s")
        self.xl_x_val.setText(f"X: {xl_x:.1f}")
        self.xl_y_val.setText(f"Y: {xl_y:.1f}")
        self.xl_z_val.setText(f"Z: {xl_z:.1f}")
        self.gy_x_val.setText(f"X: {gy_x:.1f}")
        self.gy_y_val.setText(f"Y: {gy_y:.1f}")
        self.gy_z_val.setText(f"Z: {gy_z:.1f}")
        self.hx_val.setText(f"X: {hx:.1f}")
        self.hy_val.setText(f"Y: {hy:.1f}")
        self.hz_val.setText(f"Z: {hz:.1f}")

    def _handle_continuity(self, raw: bytes):
        parsed = parse_continuity(raw)
        if parsed is None:
            self.terminal.append(f"[BAD CONT] {raw.hex()}")
            return

        def update_btn(btn, label, ok):
            btn.setText(f"{label}: {'OK' if ok else 'OPEN'}")
            color = "lime" if ok else "red"
            btn.setStyleSheet(f"background-color: {color}; color: white; font-weight: bold;")
            btn.repaint()
        update_btn(self.cont_main,   "Main",   parsed['main'])
        update_btn(self.cont_drogue, "Drogue", parsed['drogue'])

        self.terminal.append(
            f"[CONT] Main: {'OK' if parsed['main'] else 'OPEN'} | "
            f"Drogue: {'OK' if parsed['drogue'] else 'OPEN'}"
        )
