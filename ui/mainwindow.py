import sys
from PyQt5 import QtWidgets, QtCore
from collections import deque
import pyqtgraph as pg

from core.parser import parse_line, parse_telemetry, parse_continuity, build_command, CMD_ARM, CMD_FIRE, CMD_DISARM
from core.lora_worker import LoRaWorker


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Shingleback Ground Station")
        self.resize(1400, 800)

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

        gy_plot = self.plot_widget.addPlot(title="Gyro (mdps)")
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

        def make_cont_indicator(label):
            lbl = QtWidgets.QLabel(f"{label}: --")
            lbl.setStyleSheet("font-family: Courier; font-size: 13px; color: grey;")
            return lbl

        # Status
        dash_layout.addWidget(make_label("Status"))
        self.status_val = QtWidgets.QLabel("IDLE")
        self.status_val.setStyleSheet("font-size: 14px; font-weight: bold; color: orange;")
        dash_layout.addWidget(self.status_val)

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
        dash_layout.addWidget(make_label("Gyro (mdps)"))
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
        self.cont_main   = make_cont_indicator("Main")
        self.cont_drogue = make_cont_indicator("Drogue")
        dash_layout.addWidget(self.cont_main)
        dash_layout.addWidget(self.cont_drogue)

        # Commands
        dash_layout.addWidget(make_label("Commands"))
        btn_fire_drogue = QtWidgets.QPushButton("Fire Drogue")
        btn_fire_main   = QtWidgets.QPushButton("Fire Main")
        btn_fire_drogue.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")
        btn_fire_main.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")
        btn_fire_drogue.clicked.connect(lambda: self.send_command(CMD_FIRE, 1))
        btn_fire_main.clicked.connect(lambda:   self.send_command(CMD_FIRE, 2))
        dash_layout.addWidget(btn_fire_drogue)
        dash_layout.addWidget(btn_fire_main)

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
        self.worker.start()

    def send_command(self, cmd_id: int, channel: int):
        pkt = build_command(cmd_id, channel)
        self.worker.send(pkt)
        label = {1: "Drogue", 2: "Main"}.get(channel, f"ch{channel}")
        self.terminal.append(f"[CMD] FIRE {label}")
        self.terminal.ensureCursorVisible()

    def on_lora_data(self, raw: bytes):
        if len(raw) < 2:
            self.terminal.append(f"[BAD PACKET] too short: {raw.hex()}")
            return

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

        self.terminal.append(
            f"[{parsed['sequence']:03d}] "
            f"Alt: {parsed['altitude']:.2f}m | "
            f"Vel: {parsed['velocity']:.2f}m/s | "
            f"P: {parsed['pressure']:.1f}Pa | "
            f"T: {parsed['temperature']:.1f}C | "
            f"Accel X:{parsed['accel']['x']:.1f} Y:{parsed['accel']['y']:.1f} Z:{parsed['accel']['z']:.1f} | "
            f"IMU X:{parsed['imu_accel']['x']:.1f} Y:{parsed['imu_accel']['y']:.1f} Z:{parsed['imu_accel']['z']:.1f} | "
            f"Gyro X:{parsed['imu_gyro']['x']:.1f} Y:{parsed['imu_gyro']['y']:.1f} Z:{parsed['imu_gyro']['z']:.1f} | "
            f"State: {parsed['flight_state']}"
        )

        alt  = parsed['altitude']
        vel  = parsed['velocity']
        xl_x, xl_y, xl_z = parsed['accel']['x'],     parsed['accel']['y'],     parsed['accel']['z']
        gy_x, gy_y, gy_z = parsed['imu_gyro']['x'],  parsed['imu_gyro']['y'],  parsed['imu_gyro']['z']
        hx,   hy,   hz   = parsed['imu_accel']['x'], parsed['imu_accel']['y'], parsed['imu_accel']['z']

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
        self.status_val.setText("RX OK")
        self.status_val.setStyleSheet("font-size: 14px; font-weight: bold; color: green;")

    def _handle_continuity(self, raw: bytes):
        parsed = parse_continuity(raw)
        if parsed is None:
            self.terminal.append(f"[BAD CONT] {raw.hex()}")
            return

        def fmt(label, ok):
            return f"{label}: {'OK' if ok else 'OPEN'}"

        def style(ok):
            return f"font-family: Courier; font-size: 13px; color: {'lime' if ok else 'red'};"

        self.cont_main.setText(fmt("Main", parsed['main']))
        self.cont_main.setStyleSheet(style(parsed['main']))
        self.cont_drogue.setText(fmt("Drogue", parsed['drogue']))
        self.cont_drogue.setStyleSheet(style(parsed['drogue']))

        self.terminal.append(
            f"[CONT] Main: {'OK' if parsed['main'] else 'OPEN'} | "
            f"Drogue: {'OK' if parsed['drogue'] else 'OPEN'}"
        )

    def on_data(self, line):
        self.terminal.append(line)
        self.terminal.ensureCursorVisible()

        parsed = parse_line(line)
        if parsed is None:
            return

        alt, xl_x, xl_y, xl_z, gy_x, gy_y, gy_z, hx, hy, hz = parsed

        self.alt.append(alt)
        self.xl_x.append(xl_x); self.xl_y.append(xl_y); self.xl_z.append(xl_z)
        self.gy_x.append(gy_x); self.gy_y.append(gy_y); self.gy_z.append(gy_z)

        self.alt_curve.setData(list(self.alt))
        self.gy_x_curve.setData(list(self.gy_x))
        self.gy_y_curve.setData(list(self.gy_y))
        self.gy_z_curve.setData(list(self.gy_z))
        self.xl_x_curve.setData(list(self.xl_x))
        self.xl_y_curve.setData(list(self.xl_y))
        self.xl_z_curve.setData(list(self.xl_z))

        self.alt_val.setText(f"Alt: {alt:.2f} m")
        self.xl_x_val.setText(f"X: {xl_x:.1f}")
        self.xl_y_val.setText(f"Y: {xl_y:.1f}")
        self.xl_z_val.setText(f"Z: {xl_z:.1f}")
        self.gy_x_val.setText(f"X: {gy_x:.1f}")
        self.gy_y_val.setText(f"Y: {gy_y:.1f}")
        self.gy_z_val.setText(f"Z: {gy_z:.1f}")
        self.hx_val.setText(f"X: {hx:.1f}")
        self.hy_val.setText(f"Y: {hy:.1f}")
        self.hz_val.setText(f"Z: {hz:.1f}")