import random
import time
from PyQt5 import QtWidgets, QtCore, QtGui
from collections import deque
import pyqtgraph as pg

from core.parser import (
    parse_telemetry,
    parse_gps,
    parse_continuity,
    packet_type,
    build_command,
    PKT_TELEMETRY,
    PKT_GPS,
    PKT_CONTINUITY,
    CMD_ARM,
    CMD_FIRE,
    CMD_DISARM,
)
from core.lora_worker import LoRaWorker


class MainWindow(QtWidgets.QMainWindow):
    COMMAND_REPEATS = 5
    COMMAND_REPEAT_MS = 100
    ARM_CONFIRM_TIMEOUT_MS = 15000
    DISARM_CONFIRM_TIMEOUT_MS = 5000
    DISARM_TELEMETRY_GAP_SECONDS = 1.0
    DISARM_CONFIRM_POLL_MS = 50
    DISARM_MAX_RETRIES = 3
    STATE_IDLE = 0
    STATE_ARMED = 1

    FLIGHT_STATES = {
        0: ("IDLE", "#363b42", "#d7dce2"),
        1: ("PAD", "#f79322", "#08090a"),
        2: ("BOOST", "#ffd931", "#08090a"),
        3: ("COAST", "#2ad6ff", "#08090a"),
        4: ("APOGEE", "#b76cff", "#ffffff"),
        5: ("DROGUE", "#5bb4f0", "#08090a"),
        6: ("PARAFOIL", "#22e2bb", "#08090a"),
        7: ("LAND", "#36dd76", "#08090a"),
    }

    THEME = """
        QMainWindow, QWidget#root { background: #050607; color: #e8eaed; }
        QWidget { color: #e8eaed; font-family: DejaVu Sans; font-size: 11pt; }
        QFrame#header { background: #0a0c0f; border-bottom: 2px solid #f79322; }
        QLabel#brand { color: #ffffff; font-size: 20pt; font-weight: 800; letter-spacing: 1pt; }
        QLabel#subtitle { color: #f79322; font-size: 9pt; font-weight: 700; letter-spacing: 2pt; }
        QFrame#card { background: #0a0c0f; border: 1px solid #282d33; border-top: 2px solid #f79322; border-radius: 3px; }
        QLabel#sectionTitle { color: #f79322; font-family: DejaVu Sans Mono; font-size: 11pt; font-weight: 800; }
        QLabel#metricName { color: #9aa2ad; font-size: 9pt; font-weight: 700; }
        QLabel#metricValue { color: #f2f4f7; font-family: DejaVu Sans Mono; font-size: 12pt; font-weight: 700; }
        QLabel#muted { color: #8b949f; font-family: DejaVu Sans Mono; font-size: 9pt; }
        QLabel#statusChip { background: #15191e; border: 1px solid #343a42; border-radius: 3px; padding: 8px 12px; font-family: DejaVu Sans Mono; font-size: 10pt; font-weight: 700; }
        QLineEdit { background: #050607; border: 1px solid #343a42; border-radius: 3px; color: #ffffff; padding: 10px; font-family: DejaVu Sans Mono; font-size: 10pt; }
        QLineEdit:focus { border-color: #f79322; }
        QPushButton { background: #171b20; border: 1px solid #3b424b; border-radius: 3px; color: #f2f4f7; padding: 10px 14px; font-size: 10pt; font-weight: 700; }
        QPushButton:hover { border-color: #f79322; background: #22272e; }
        QPushButton:pressed { background: #0f1215; }
        QPushButton:disabled { color: #555d66; border-color: #282d33; background: #0c0e11; }
        QPushButton#danger { background: #4a1015; border-color: #b92b38; color: #ffffff; }
        QPushButton#danger:hover { background: #681821; border-color: #ff5261; }
        QPushButton#primary { background: #6a3e0b; border-color: #f79322; color: #ffffff; }
        QTextEdit { background: #030405; border: 1px solid #282d33; color: #cdd3da; font-family: DejaVu Sans Mono; font-size: 10pt; padding: 8px; }
        QScrollArea { border: 0; background: transparent; }
        QSplitter::handle { background: #171b20; }
        QSplitter::handle:hover { background: #f79322; }
        QScrollBar:vertical { background: #090b0d; width: 14px; }
        QScrollBar::handle:vertical { background: #343a42; min-height: 28px; border-radius: 4px; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Shingleback · ODIN Ground Control")
        self.resize(1540, 940)
        self.setMinimumSize(1100, 720)
        self.setStyleSheet(self.THEME)

        # Arm state
        self.arm_code = str(random.randint(1000, 9999))
        self.armed = False
        self.pending_arm = False
        self.pending_disarm = False
        self.arm_burst_sent = False
        self.disarm_burst_sent = False
        self.fire_code = None
        self.disarm_requested_at = 0.0
        self.last_telemetry_at = 0.0
        self.last_continuity_at = 0.0
        self.flight_state = 0
        self.packet_count = 0
        self.gps_packet_count = 0

        n = 100
        self.alt = deque(maxlen=n)
        self.gy_x = deque(maxlen=n)
        self.gy_y = deque(maxlen=n)
        self.gy_z = deque(maxlen=n)
        self.xl_x = deque(maxlen=n)
        self.xl_y = deque(maxlen=n)
        self.xl_z = deque(maxlen=n)
        self.vel = deque(maxlen=n)

        root = QtWidgets.QWidget()
        root.setObjectName("root")
        root_layout = QtWidgets.QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        root_layout.addWidget(self._build_header())

        body = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        body.setHandleWidth(3)
        body.addWidget(self._build_data_workspace())
        body.addWidget(self._build_dashboard())
        body.setSizes([1040, 500])
        body.setStretchFactor(0, 1)
        body.setStretchFactor(1, 0)
        root_layout.addWidget(body, 1)
        self.setCentralWidget(root)

        self.update_flight_state_indicator()
        self._set_link_state(False)

        # Worker
        self.worker = LoRaWorker()
        self.worker.data_received.connect(self.on_lora_data)
        self.worker.error_occurred.connect(self._handle_radio_error)

        self.arm_timeout_timer = QtCore.QTimer(self)
        self.arm_timeout_timer.setSingleShot(True)
        self.arm_timeout_timer.timeout.connect(self.handle_arm_timeout)

        self.disarm_confirm_timer = QtCore.QTimer(self)
        self.disarm_confirm_timer.setInterval(self.DISARM_CONFIRM_POLL_MS)
        self.disarm_confirm_timer.timeout.connect(self.check_disarm_confirmation)

        self.disarm_timeout_timer = QtCore.QTimer(self)
        self.disarm_timeout_timer.setSingleShot(True)
        self.disarm_timeout_timer.timeout.connect(self.handle_disarm_timeout)
        self.worker.start()

    def _build_header(self):
        header = QtWidgets.QFrame()
        header.setObjectName("header")
        header.setFixedHeight(96)
        layout = QtWidgets.QHBoxLayout(header)
        layout.setContentsMargins(20, 10, 20, 10)
        layout.setSpacing(14)

        brand_box = QtWidgets.QVBoxLayout()
        brand_box.setSpacing(0)
        brand = QtWidgets.QLabel("SHINGLEBACK")
        brand.setObjectName("brand")
        subtitle = QtWidgets.QLabel("ODIN  ·  GROUND CONTROL")
        subtitle.setObjectName("subtitle")
        brand_box.addWidget(brand)
        brand_box.addWidget(subtitle)
        layout.addLayout(brand_box)
        layout.addStretch()

        self.radio_status = QtWidgets.QLabel("●  RADIO WAITING")
        self.radio_status.setObjectName("statusChip")
        self.radio_status.setToolTip(
            "915 MHz · SF7 · 125 kHz · CR 4/5 · explicit header · CRC · sync 0x12 · preamble 8"
        )
        self.gps_header_status = QtWidgets.QLabel("●  GPS NO DATA")
        self.gps_header_status.setObjectName("statusChip")
        self.header_state = QtWidgets.QLabel("IDLE")
        self.header_state.setObjectName("statusChip")
        layout.addWidget(self.radio_status)
        layout.addWidget(self.gps_header_status)
        layout.addWidget(self.header_state)

        reset_btn = QtWidgets.QPushButton("RESET DISPLAY")
        reset_btn.setToolTip("Clear plots, displayed readings, GPS status, continuity, and event log")
        reset_btn.clicked.connect(self.clear_display)
        layout.addWidget(reset_btn)
        return header

    def _configure_plot(self, plot, title, left_label):
        plot.setTitle(
            f"<span style='color:#f79322;font-size:15pt;font-weight:700'>"
            f"&lt; {title} &gt;</span>"
        )
        plot.setLabel(
            "left",
            left_label,
            color="#b4bbc4",
            **{"font-size": "11pt", "font-weight": "700"},
        )
        plot.setLabel(
            "bottom",
            "Samples",
            color="#89919c",
            **{"font-size": "10pt"},
        )
        plot.showGrid(x=True, y=True, alpha=0.16)
        plot.setMenuEnabled(False)
        axis_font = QtGui.QFont("DejaVu Sans Mono", 10)
        for axis_name in ("left", "bottom"):
            axis = plot.getAxis(axis_name)
            axis.setPen(pg.mkPen("#59616c"))
            axis.setTextPen(pg.mkPen("#aab1ba"))
            axis.setStyle(tickFont=axis_font)

    def _build_data_workspace(self):
        workspace = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(workspace)
        layout.setContentsMargins(12, 12, 8, 12)
        layout.setSpacing(8)

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground("#07090b")

        alt_plot = self.plot_widget.addPlot(row=0, col=0)
        self._configure_plot(alt_plot, "ALTITUDE", "Altitude (m)")
        self.alt_curve = alt_plot.plot(pen=pg.mkPen("#f63356", width=2))

        vel_plot = self.plot_widget.addPlot(row=0, col=1)
        self._configure_plot(vel_plot, "VERTICAL VELOCITY", "Velocity (m/s)")
        self.vel_curve = vel_plot.plot(pen=pg.mkPen("#f79322", width=2))

        gy_plot = self.plot_widget.addPlot(row=1, col=0)
        self._configure_plot(gy_plot, "ANGULAR VELOCITY · GYROSCOPE", "Rotation rate (dps)")
        gy_plot.addLegend(offset=(10, 10), labelTextSize="10pt")
        self.gy_x_curve = gy_plot.plot(pen=pg.mkPen("#f63356", width=1.5), name="X")
        self.gy_y_curve = gy_plot.plot(pen=pg.mkPen("#36dd76", width=1.5), name="Y")
        self.gy_z_curve = gy_plot.plot(pen=pg.mkPen("#5bb4f0", width=1.5), name="Z")

        xl_plot = self.plot_widget.addPlot(row=1, col=1)
        self._configure_plot(xl_plot, "LINEAR ACCELERATION · IMU", "Acceleration (mg)")
        xl_plot.addLegend(offset=(10, 10), labelTextSize="10pt")
        self.xl_x_curve = xl_plot.plot(pen=pg.mkPen("#f63356", width=1.5), name="X")
        self.xl_y_curve = xl_plot.plot(pen=pg.mkPen("#36dd76", width=1.5), name="Y")
        self.xl_z_curve = xl_plot.plot(pen=pg.mkPen("#5bb4f0", width=1.5), name="Z")

        self.terminal = QtWidgets.QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setPlaceholderText("Radio events and decoded packets appear here…")
        self.terminal.document().setMaximumBlockCount(1500)

        terminal_bar = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("EVENT LOG")
        title.setObjectName("sectionTitle")
        self.packet_counter = QtWidgets.QLabel("0 PACKETS")
        self.packet_counter.setObjectName("muted")
        terminal_bar.addWidget(title)
        terminal_bar.addStretch()
        terminal_bar.addWidget(self.packet_counter)

        plot_terminal_split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        plot_terminal_split.setHandleWidth(3)
        plot_terminal_split.addWidget(self.plot_widget)
        terminal_container = QtWidgets.QWidget()
        terminal_layout = QtWidgets.QVBoxLayout(terminal_container)
        terminal_layout.setContentsMargins(0, 0, 0, 0)
        terminal_layout.setSpacing(5)
        terminal_layout.addLayout(terminal_bar)
        terminal_layout.addWidget(self.terminal)
        plot_terminal_split.addWidget(terminal_container)
        plot_terminal_split.setSizes([610, 220])
        layout.addWidget(plot_terminal_split)
        return workspace

    def _card(self, title):
        card = QtWidgets.QFrame()
        card.setObjectName("card")
        layout = QtWidgets.QVBoxLayout(card)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(8)
        heading = QtWidgets.QLabel(title)
        heading.setObjectName("sectionTitle")
        layout.addWidget(heading)
        return card, layout

    def _metric(self, name, initial="--"):
        box = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(box)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(1)
        label = QtWidgets.QLabel(name.upper())
        label.setObjectName("metricName")
        value = QtWidgets.QLabel(initial)
        value.setObjectName("metricValue")
        value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(label)
        layout.addWidget(value)
        return box, value

    def _metric_grid(self, items, columns=2):
        grid = QtWidgets.QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(9)
        for index, (name, attribute, initial) in enumerate(items):
            widget, value = self._metric(name, initial)
            setattr(self, attribute, value)
            grid.addWidget(widget, index // columns, index % columns)
        return grid

    def _indicator(self, label):
        indicator = QtWidgets.QLabel(f"●  {label}: --")
        indicator.setObjectName("statusChip")
        indicator.setAlignment(QtCore.Qt.AlignCenter)
        return indicator

    def _build_dashboard(self):
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(460)
        scroll.setMaximumWidth(540)
        dashboard = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(dashboard)
        layout.setContentsMargins(8, 12, 12, 12)
        layout.setSpacing(9)

        flight_card, flight_layout = self._card("FLIGHT COMPUTER")
        self.flight_state_val = QtWidgets.QLabel()
        self.flight_state_val.setAlignment(QtCore.Qt.AlignCenter)
        self.flight_state_val.setMinimumHeight(50)
        flight_layout.addWidget(self.flight_state_val)
        flight_layout.addLayout(self._metric_grid([
            ("Altitude", "alt_val", "-- m"),
            ("Velocity", "vel_val", "-- m/s"),
            ("IMU accel X", "xl_x_val", "-- mg"),
            ("IMU accel Y", "xl_y_val", "-- mg"),
            ("IMU accel Z", "xl_z_val", "-- mg"),
            ("H3LIS X", "hx_val", "-- mg"),
            ("H3LIS Y", "hy_val", "-- mg"),
            ("H3LIS Z", "hz_val", "-- mg"),
            ("Gyro X", "gy_x_val", "-- dps"),
            ("Gyro Y", "gy_y_val", "-- dps"),
            ("Gyro Z", "gy_z_val", "-- dps"),
        ]))
        layout.addWidget(flight_card)

        gps_card, gps_layout = self._card("GPS RECEIVER")
        uart_info = QtWidgets.QLabel("UART5  ·  9600 BAUD  ·  8-N-1  |  TX → PD2 / RX ← PC12")
        uart_info.setObjectName("muted")
        uart_info.setWordWrap(True)
        gps_layout.addWidget(uart_info)
        self.gps_fix_status = self._indicator("FIX")
        gps_layout.addWidget(self.gps_fix_status)
        gps_layout.addLayout(self._metric_grid([
            ("Latitude", "gps_lat_val", "--°"),
            ("Longitude", "gps_lon_val", "--°"),
            ("GPS altitude", "gps_alt_val", "-- m MSL"),
            ("Ground speed", "gps_speed_val", "-- m/s"),
            ("Satellites", "gps_sat_val", "--"),
            ("Fix quality", "gps_quality_val", "--"),
            ("Course", "gps_course_val", "--°"),
            ("UTC", "gps_utc_val", "--:--:--.---"),
            ("Data age", "gps_age_val", "-- ms"),
            ("Sequence", "gps_seq_val", "--"),
        ]))
        flags = QtWidgets.QHBoxLayout()
        self.gps_gga_status = self._indicator("GGA")
        self.gps_rmc_status = self._indicator("RMC")
        self.gps_uart_status = self._indicator("UART")
        flags.addWidget(self.gps_gga_status)
        flags.addWidget(self.gps_rmc_status)
        flags.addWidget(self.gps_uart_status)
        gps_layout.addLayout(flags)
        layout.addWidget(gps_card)

        continuity_card, continuity_layout = self._card("PYRO CONTINUITY")
        self.cont_main = self._indicator("MAIN")
        self.cont_drogue = self._indicator("DROGUE")
        continuity_layout.addWidget(self.cont_main)
        continuity_layout.addWidget(self.cont_drogue)
        layout.addWidget(continuity_card)

        command_card, command_layout = self._card("FLIGHT COMMANDS")
        code_row = QtWidgets.QHBoxLayout()
        code_label = QtWidgets.QLabel("ARM CODE")
        code_label.setObjectName("metricName")
        self.arm_code_display = QtWidgets.QLabel(self.arm_code)
        self.arm_code_display.setObjectName("metricValue")
        code_row.addWidget(code_label)
        code_row.addStretch()
        code_row.addWidget(self.arm_code_display)
        command_layout.addLayout(code_row)

        self.arm_input = QtWidgets.QLineEdit()
        self.arm_input.setPlaceholderText("Enter code to arm")
        self.arm_input.setMaxLength(4)
        command_layout.addWidget(self.arm_input)
        button_row = QtWidgets.QHBoxLayout()
        self.arm_btn = QtWidgets.QPushButton("ARM")
        self.arm_btn.setObjectName("primary")
        self.arm_btn.clicked.connect(self.try_arm)
        self.disarm_btn = QtWidgets.QPushButton("DISARM")
        self.disarm_btn.setEnabled(False)
        self.disarm_btn.clicked.connect(lambda: self.do_disarm(send_remote=True))
        button_row.addWidget(self.arm_btn)
        button_row.addWidget(self.disarm_btn)
        command_layout.addLayout(button_row)

        self.fire_code_display = QtWidgets.QLabel("Fire Code: --")
        self.fire_code_display.setObjectName("muted")
        self.fire_input = QtWidgets.QLineEdit()
        self.fire_input.setPlaceholderText("Enter code to fire")
        self.fire_input.setMaxLength(4)
        self.fire_input.setEnabled(False)
        self.btn_fire_drogue = QtWidgets.QPushButton("FIRE DROGUE")
        self.btn_fire_main = QtWidgets.QPushButton("FIRE MAIN")
        self.btn_fire_drogue.setObjectName("danger")
        self.btn_fire_main.setObjectName("danger")
        self.btn_fire_drogue.setEnabled(False)
        self.btn_fire_main.setEnabled(False)
        self.btn_fire_drogue.clicked.connect(lambda: self.send_command(CMD_FIRE, 1))
        self.btn_fire_main.clicked.connect(lambda: self.send_command(CMD_FIRE, 2))
        command_layout.addWidget(self.fire_code_display)
        command_layout.addWidget(self.fire_input)
        fire_row = QtWidgets.QHBoxLayout()
        fire_row.addWidget(self.btn_fire_drogue)
        fire_row.addWidget(self.btn_fire_main)
        command_layout.addLayout(fire_row)
        layout.addWidget(command_card)
        layout.addStretch()

        scroll.setWidget(dashboard)
        return scroll

    def _set_indicator(self, widget, label, state, *, fault=False):
        if state is None:
            text = "--"
            color = "#77808b"
            border = "#343a42"
            background = "#15191e"
        elif fault:
            text = "ERROR" if state else "OK"
            color = "#ff6573" if state else "#36dd76"
            border = "#9e2430" if state else "#197442"
            background = "#2a0d11" if state else "#092016"
        else:
            text = "OK" if state else "NO"
            color = "#36dd76" if state else "#ff6573"
            border = "#197442" if state else "#9e2430"
            background = "#092016" if state else "#2a0d11"
        widget.setText(f"●  {label}: {text}")
        widget.setStyleSheet(
            f"color: {color}; background: {background}; border: 1px solid {border}; "
            "border-radius: 3px; padding: 6px 8px; font-family: DejaVu Sans Mono; font-weight: 700;"
        )

    def _set_link_state(self, receiving, *, error=False):
        if error:
            self.radio_status.setText("●  RADIO ERROR")
            color, border, background = "#ff6573", "#9e2430", "#2a0d11"
        elif receiving:
            self.radio_status.setText("●  RADIO RECEIVING")
            color, border, background = "#36dd76", "#197442", "#092016"
        else:
            self.radio_status.setText("●  RADIO WAITING")
            color, border, background = "#f79322", "#6a3e0b", "#211507"
        self.radio_status.setStyleSheet(
            f"color: {color}; background: {background}; border: 1px solid {border}; "
            "border-radius: 3px; padding: 6px 10px; font-family: DejaVu Sans Mono; font-weight: 700;"
        )

    def _handle_radio_error(self, message):
        self._set_link_state(False, error=True)
        self.terminal.append(f"[ERROR] {message}")

    @staticmethod
    def _format_utc(utc_ms):
        utc_ms %= 86_400_000
        hours, remainder = divmod(utc_ms, 3_600_000)
        minutes, remainder = divmod(remainder, 60_000)
        seconds, milliseconds = divmod(remainder, 1000)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}.{milliseconds:03d}"

    def clear_display(self):
        """Clear displayed history without stopping the radio or changing arm state."""
        for series in (
            self.alt, self.vel,
            self.gy_x, self.gy_y, self.gy_z,
            self.xl_x, self.xl_y, self.xl_z,
        ):
            series.clear()
        for curve in (
            self.alt_curve, self.vel_curve,
            self.gy_x_curve, self.gy_y_curve, self.gy_z_curve,
            self.xl_x_curve, self.xl_y_curve, self.xl_z_curve,
        ):
            curve.setData([])

        for value, placeholder in (
            (self.alt_val, "-- m"),
            (self.vel_val, "-- m/s"),
            (self.xl_x_val, "-- mg"),
            (self.xl_y_val, "-- mg"),
            (self.xl_z_val, "-- mg"),
            (self.gy_x_val, "-- dps"),
            (self.gy_y_val, "-- dps"),
            (self.gy_z_val, "-- dps"),
            (self.hx_val, "-- mg"),
            (self.hy_val, "-- mg"),
            (self.hz_val, "-- mg"),
            (self.gps_lat_val, "--°"),
            (self.gps_lon_val, "--°"),
            (self.gps_alt_val, "-- m MSL"),
            (self.gps_speed_val, "-- m/s"),
            (self.gps_sat_val, "--"),
            (self.gps_quality_val, "--"),
            (self.gps_course_val, "--°"),
            (self.gps_utc_val, "--:--:--.---"),
            (self.gps_age_val, "-- ms"),
            (self.gps_seq_val, "--"),
        ):
            value.setText(placeholder)

        self._set_indicator(self.cont_main, "MAIN", None)
        self._set_indicator(self.cont_drogue, "DROGUE", None)
        self._set_indicator(self.gps_fix_status, "FIX", None)
        self._set_indicator(self.gps_gga_status, "GGA", None)
        self._set_indicator(self.gps_rmc_status, "RMC", None)
        self._set_indicator(self.gps_uart_status, "UART", None, fault=True)
        self.gps_header_status.setText("●  GPS NO DATA")
        self.gps_header_status.setStyleSheet("")
        self.packet_count = 0
        self.gps_packet_count = 0
        self.packet_counter.setText("0 PACKETS")
        self._set_link_state(False)
        self.terminal.clear()

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
        self.pending_disarm = False
        self.arm_burst_sent = False
        self.disarm_burst_sent = False
        self.disarm_confirm_timer.stop()
        self.disarm_timeout_timer.stop()
        self.reset_fire_code()
        self.update_command_buttons()
        self.arm_btn.setEnabled(False)
        self.arm_input.setEnabled(False)
        self.arm_code_display.setText("ARMING...")
        self.arm_code_display.setStyleSheet(
            "font-family: DejaVu Sans Mono; font-size: 14pt; font-weight: bold; color: #f79322;"
        )
        self.terminal.append("[SYS] ARM REQUEST QUEUED - waiting for continuity packet")
        self.terminal.ensureCursorVisible()

        self.arm_timeout_timer.start(self.ARM_CONFIRM_TIMEOUT_MS)

        if self.flight_state == self.STATE_ARMED:
            self.confirm_arm()

    def send_arm_request(self):
        if not self.pending_arm:
            return
        if self.flight_state == self.STATE_ARMED:
            self.confirm_arm()
            return
        self.arm_burst_sent = True
        self.terminal.append("[SYS] CONTINUITY RECEIVED - sending ARM burst")
        self.terminal.ensureCursorVisible()
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
        self.arm_burst_sent = False
        self.arm_timeout_timer.stop()
        self.armed = True
        self.pending_disarm = False
        self.disarm_confirm_timer.stop()
        self.disarm_timeout_timer.stop()
        self.generate_fire_code()
        self.update_command_buttons()
        self.arm_btn.setEnabled(False)
        self.arm_input.setEnabled(False)
        self.arm_code_display.setText("ARMED")
        self.arm_code_display.setStyleSheet(
            "font-family: DejaVu Sans Mono; font-size: 14pt; font-weight: bold; color: #36dd76;"
        )
        self.terminal.append("[SYS] ARM CONFIRMED - flight computer reported ARMED")
        self.terminal.ensureCursorVisible()

    def handle_arm_timeout(self):
        if not self.pending_arm:
            return

        self.pending_arm = False
        arm_was_sent = self.arm_burst_sent
        self.armed = False
        self.arm_burst_sent = False
        self.reset_fire_code()
        self.update_command_buttons()
        self.arm_btn.setEnabled(True)
        self.arm_input.setEnabled(True)
        self.arm_input.clear()
        self.arm_input.setPlaceholderText("Enter code to arm")
        self.arm_input.setStyleSheet("")
        self.arm_code_display.setText(self.arm_code)
        self.arm_code_display.setStyleSheet(
            "font-family: DejaVu Sans Mono; font-size: 16pt; font-weight: bold; color: #ff6573;"
        )
        if arm_was_sent:
            self.terminal.append("[SYS] ARM TIMEOUT - ARMED telemetry not received")
        else:
            self.terminal.append("[SYS] ARM TIMEOUT - no continuity packet received")
        self.terminal.ensureCursorVisible()

    def air_packet_length(self, packet: bytes) -> int:
        return len(packet) + LoRaWorker.RADIOHEAD_HEADER_LENGTH

    def queue_command(
        self,
        cmd_id: int,
        channel: int,
        label: str,
        repeats: int | None = None,
        should_send=None,
        repeat_ms: int | None = None,
    ):
        if repeats is None:
            repeats = self.COMMAND_REPEATS
        if repeat_ms is None:
            repeat_ms = self.COMMAND_REPEAT_MS

        pkt = build_command(cmd_id, channel)
        self.terminal.append(
            f"[CMD] {label} ({repeats} packets, payload={len(pkt)}B, air={self.air_packet_length(pkt)}B)"
        )
        self.terminal.ensureCursorVisible()
        self.send_queued_packet(pkt, should_send)
        for repeat in range(1, repeats):
            QtCore.QTimer.singleShot(
                repeat * repeat_ms,
                lambda packet=pkt, predicate=should_send: self.send_queued_packet(
                    packet,
                    predicate,
                ),
            )

    def send_queued_packet(self, packet: bytes, should_send=None):
        if should_send is None or should_send():
            self.worker.send(packet)

    def update_flight_state_indicator(self):
        name, bg_color, fg_color = self.FLIGHT_STATES.get(
            self.flight_state,
            (f"UNKNOWN {self.flight_state}", "red", "white"),
        )
        self.flight_state_val.setText(name)
        self.flight_state_val.setStyleSheet(
            "font-family: DejaVu Sans Mono; font-size: 16pt; font-weight: 800; padding: 12px 8px; "
            f"background-color: {bg_color}; color: {fg_color}; border-radius: 3px;"
        )
        self.header_state.setText(name)
        self.header_state.setStyleSheet(
            f"color: {fg_color}; background: {bg_color}; border: 1px solid {bg_color}; "
            "border-radius: 3px; padding: 6px 10px; font-family: DejaVu Sans Mono; font-weight: 800;"
        )

    def update_command_buttons(self):
        fire_enabled = (
            self.armed
            and self.flight_state == self.STATE_ARMED
            and not self.pending_disarm
        )
        disarm_enabled = (
            self.pending_arm
            or self.pending_disarm
            or self.flight_state == self.STATE_ARMED
        )
        self.btn_fire_drogue.setEnabled(fire_enabled)
        self.btn_fire_main.setEnabled(fire_enabled)
        self.fire_input.setEnabled(fire_enabled)
        self.disarm_btn.setEnabled(disarm_enabled)

    def generate_fire_code(self):
        self.fire_code = str(random.randint(1000, 9999))
        self.fire_code_display.setText(f"Fire Code: {self.fire_code}")
        self.fire_input.clear()
        self.fire_input.setPlaceholderText("Enter code to fire")
        self.fire_input.setStyleSheet("")

    def reset_fire_code(self):
        self.fire_code = None
        self.fire_code_display.setText("Fire Code: --")
        self.fire_input.clear()
        self.fire_input.setPlaceholderText("Enter code to fire")
        self.fire_input.setStyleSheet("")
        self.fire_input.setEnabled(False)

    def validate_fire_code(self, channel: int, label: str) -> bool:
        # Fire code authentication removed — allow firing while armed
        return True

    def do_disarm(self, *, send_remote: bool = True):
        if send_remote:
            self.begin_disarm_request()
            return

        self.pending_arm = False
        self.pending_disarm = False
        self.arm_burst_sent = False
        self.disarm_burst_sent = False
        self.disarm_confirm_timer.stop()
        self.disarm_timeout_timer.stop()
        self.arm_timeout_timer.stop()
        self.armed = False
        self.reset_arm_controls("[SYS] DISARMED")

    def begin_disarm_request(self):
        self.pending_arm = False
        self.pending_disarm = True
        self.arm_burst_sent = False
        self.disarm_burst_sent = False
        self.disarm_attempts = 0
        self.arm_timeout_timer.stop()
        self.reset_fire_code()
        self.disarm_requested_at = time.monotonic()
        self.last_telemetry_at = self.disarm_requested_at
        self.last_continuity_at = 0.0
        self.update_command_buttons()
        self.arm_btn.setEnabled(False)
        self.arm_input.setEnabled(False)
        self.arm_code_display.setText("DISARMING...")
        self.arm_code_display.setStyleSheet(
            "font-family: DejaVu Sans Mono; font-size: 14pt; font-weight: bold; color: #f79322;"
        )
        self.terminal.append("[SYS] DISARM REQUEST QUEUED - waiting for continuity packet")
        self.terminal.ensureCursorVisible()
        self.disarm_timeout_timer.start(self.DISARM_CONFIRM_TIMEOUT_MS)

    def send_disarm_request(self):
        if not self.pending_disarm:
            return

        # allow repeated attempts; increment attempt counter
        self.disarm_attempts = getattr(self, 'disarm_attempts', 0) + 1
        self.disarm_burst_sent = True
        self.disarm_requested_at = time.monotonic()
        self.last_telemetry_at = self.disarm_requested_at
        self.terminal.append(f"[SYS] CONTINUITY RECEIVED - sending DISARM burst (attempt {self.disarm_attempts})")
        self.terminal.ensureCursorVisible()
        # send a dense burst to increase chance of hitting the flight controller's
        # short RX window (flight polls ~every 200ms and uses a 50ms RX timeout)
        self.queue_command(
            CMD_DISARM,
            0,
            "DISARM",
            repeats=12,
            repeat_ms=25,
            should_send=lambda: self.pending_disarm,
        )
        self.disarm_confirm_timer.start()
        self.disarm_timeout_timer.start(self.DISARM_CONFIRM_TIMEOUT_MS)

    def confirm_disarm(self):
        if not self.pending_disarm:
            return

        self.pending_disarm = False
        self.disarm_burst_sent = False
        self.disarm_confirm_timer.stop()
        self.disarm_timeout_timer.stop()
        self.armed = False
        self.flight_state = self.STATE_IDLE
        self.update_flight_state_indicator()
        self.reset_fire_code()
        self.reset_arm_controls("[SYS] DISARM CONFIRMED - continuity received and telemetry stopped")

    def handle_disarm_timeout(self):
        if not self.pending_disarm:
            return
        # If we haven't exceeded retry count, try another dense burst
        self.disarm_confirm_timer.stop()
        self.disarm_timeout_timer.stop()
        attempts = getattr(self, 'disarm_attempts', 0)
        if attempts < self.DISARM_MAX_RETRIES and self.pending_disarm:
            self.terminal.append(f"[SYS] DISARM attempt {attempts} timed out — retrying")
            self.terminal.ensureCursorVisible()
            # allow another burst to be sent
            self.disarm_burst_sent = False
            QtCore.QTimer.singleShot(200, self.send_disarm_request)
            return

        # give up
        self.pending_disarm = False
        disarm_was_sent = self.disarm_burst_sent
        self.disarm_burst_sent = False
        if disarm_was_sent:
            self.terminal.append("[SYS] DISARM NOT CONFIRMED - telemetry still active or no continuity after retries")
        else:
            self.terminal.append("[SYS] DISARM NOT SENT - no continuity packet received")
        self.terminal.ensureCursorVisible()

        if self.armed and self.flight_state == self.STATE_ARMED:
            self.generate_fire_code()
            self.update_command_buttons()
            self.arm_code_display.setText("ARMED")
            self.arm_code_display.setStyleSheet(
                "font-family: DejaVu Sans Mono; font-size: 14pt; font-weight: bold; color: #36dd76;"
            )
            self.arm_btn.setEnabled(False)
            self.arm_input.setEnabled(False)
        else:
            self.update_command_buttons()
            self.arm_btn.setEnabled(True)
            self.arm_input.setEnabled(True)

    def check_disarm_confirmation(self):
        if not self.pending_disarm or not self.disarm_burst_sent:
            return

        now = time.monotonic()
        continuity_after_disarm = self.last_continuity_at > self.disarm_requested_at
        telemetry_quiet = (
            now - self.last_telemetry_at
        ) >= self.DISARM_TELEMETRY_GAP_SECONDS

        # confirm if we saw continuity after the request and telemetry has stopped,
        # or if a telemetry packet reports the flight_state changed to IDLE
        if (continuity_after_disarm and telemetry_quiet) or (self.flight_state == self.STATE_IDLE):
            self.confirm_disarm()

    def reset_arm_controls(self, message: str):
        self.arm_burst_sent = False
        self.disarm_burst_sent = False
        self.reset_fire_code()
        self.update_command_buttons()
        self.arm_btn.setEnabled(True)
        self.arm_input.setEnabled(True)
        self.arm_input.clear()
        self.arm_input.setPlaceholderText("Enter code to arm")
        self.arm_input.setStyleSheet("")
        self.arm_code = str(random.randint(1000, 9999))
        self.arm_code_display.setText(self.arm_code)
        self.arm_code_display.setStyleSheet(
            "font-family: DejaVu Sans Mono; font-size: 16pt; font-weight: bold; color: #ff6573;"
        )
        self.terminal.append(message)
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
        if not self.validate_fire_code(channel, label):
            return

        self.queue_command(
            cmd_id,
            channel,
            f"FIRE {label}",
            should_send=lambda: self.flight_state == self.STATE_ARMED,
        )
        # Do NOT auto-disarm after firing — keep system armed so operator can
        # send multiple FIRE commands. Manual disarm remains available.

    def on_lora_data(self, raw: bytes):
        self.packet_count += 1
        self.packet_counter.setText(
            f"{self.packet_count} PACKETS  ·  {self.gps_packet_count} GPS"
        )
        self._set_link_state(True)

        pkt_type = packet_type(raw)
        if pkt_type is None:
            self.terminal.append(
                f"[BAD PACKET] invalid receiver header/sync len={len(raw)} {raw.hex()}"
            )
            return

        self.terminal.append(f"[RAW] type={pkt_type:#04x} len={len(raw)}")

        if pkt_type == PKT_TELEMETRY:
            self._handle_telemetry(raw)
        elif pkt_type == PKT_GPS:
            self._handle_gps(raw)
        elif pkt_type == PKT_CONTINUITY:
            self._handle_continuity(raw)
        else:
            self.terminal.append(f"[UNKNOWN PKT] type={pkt_type:#04x} {raw.hex()}")
        self.terminal.ensureCursorVisible()

    def _handle_telemetry(self, raw: bytes):
        parsed = parse_telemetry(raw)
        if parsed is None:
            self.terminal.append(f"[BAD TELEM] {raw.hex()}")
            return

        self.last_telemetry_at = time.monotonic()
        self.flight_state = parsed['flight_state']
        self.update_flight_state_indicator()
        if self.pending_arm and self.flight_state == self.STATE_ARMED:
            self.confirm_arm()
        self.update_command_buttons()

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

        self.alt_val.setText(f"{alt:.2f} m")
        self.vel_val.setText(f"{vel:.2f} m/s")
        self.xl_x_val.setText(f"{xl_x:.1f} mg")
        self.xl_y_val.setText(f"{xl_y:.1f} mg")
        self.xl_z_val.setText(f"{xl_z:.1f} mg")
        self.gy_x_val.setText(f"{gy_x:.1f} dps")
        self.gy_y_val.setText(f"{gy_y:.1f} dps")
        self.gy_z_val.setText(f"{gy_z:.1f} dps")
        self.hx_val.setText(f"{hx:.1f} mg")
        self.hy_val.setText(f"{hy:.1f} mg")
        self.hz_val.setText(f"{hz:.1f} mg")

    def _handle_gps(self, raw: bytes):
        parsed = parse_gps(raw)
        if parsed is None:
            self.gps_header_status.setText("●  GPS PACKET ERROR")
            self.gps_header_status.setStyleSheet(
                "color: #ff6573; background: #2a0d11; border: 1px solid #9e2430; "
                "border-radius: 3px; padding: 6px 10px; font-family: DejaVu Sans Mono; font-weight: 700;"
            )
            self.terminal.append(f"[BAD GPS] length/header/type/CRC check failed: {raw.hex()}")
            return

        self.gps_packet_count += 1
        self.packet_counter.setText(
            f"{self.packet_count} PACKETS  ·  {self.gps_packet_count} GPS"
        )
        self.flight_state = parsed["flight_state"]
        self.update_flight_state_indicator()
        if self.pending_arm and self.flight_state == self.STATE_ARMED:
            self.confirm_arm()
        self.update_command_buttons()

        fix_valid = parsed["fix_valid"]
        self._set_indicator(self.gps_fix_status, "FIX", fix_valid)
        self._set_indicator(self.gps_gga_status, "GGA", parsed["gga_received"])
        self._set_indicator(self.gps_rmc_status, "RMC", parsed["rmc_active"])
        self._set_indicator(self.gps_uart_status, "UART", parsed["uart_error"], fault=True)

        if fix_valid:
            self.gps_header_status.setText("●  GPS FIX VALID")
            color, border, background = "#36dd76", "#197442", "#092016"
        else:
            self.gps_header_status.setText("●  GPS NO FIX")
            color, border, background = "#ffd166", "#80641d", "#251d08"
        self.gps_header_status.setStyleSheet(
            f"color: {color}; background: {background}; border: 1px solid {border}; "
            "border-radius: 3px; padding: 6px 10px; font-family: DejaVu Sans Mono; font-weight: 700;"
        )

        self.gps_lat_val.setText(f"{parsed['latitude']:.7f}°")
        self.gps_lon_val.setText(f"{parsed['longitude']:.7f}°")
        self.gps_alt_val.setText(f"{parsed['altitude_m']:.3f} m MSL")
        self.gps_speed_val.setText(f"{parsed['ground_speed_mps']:.2f} m/s")
        self.gps_sat_val.setText(str(parsed["satellites"]))
        self.gps_quality_val.setText(str(parsed["fix_quality"]))
        self.gps_course_val.setText(f"{parsed['course_deg']:.2f}°")
        self.gps_utc_val.setText(self._format_utc(parsed["utc_ms"]))
        if parsed["information_age_ms"] == 65535:
            self.gps_age_val.setText("≥ 65.535 s")
        else:
            self.gps_age_val.setText(f"{parsed['information_age_ms']} ms")
        self.gps_seq_val.setText(str(parsed["sequence"]))

        self.terminal.append(
            f"[GPS {parsed['sequence']:03d}] "
            f"Fix: {'VALID' if fix_valid else 'NO'} | Q:{parsed['fix_quality']} | "
            f"Sats:{parsed['satellites']} | Lat:{parsed['latitude']:.7f} | "
            f"Lon:{parsed['longitude']:.7f} | Alt:{parsed['altitude_m']:.3f}m MSL | "
            f"Speed:{parsed['ground_speed_mps']:.2f}m/s | Course:{parsed['course_deg']:.2f}° | "
            f"Age:{parsed['information_age_ms']}ms | State:{parsed['flight_state_name']}"
        )

    def _handle_continuity(self, raw: bytes):
        parsed = parse_continuity(raw)
        if parsed is None:
            self.terminal.append(f"[BAD CONT] {raw.hex()}")
            return
        self.last_continuity_at = time.monotonic()
        if self.pending_arm and not self.arm_burst_sent:
            self.send_arm_request()
        if self.pending_disarm and not self.disarm_burst_sent:
            self.send_disarm_request()
        self.check_disarm_confirmation()

        self._set_indicator(self.cont_main, "MAIN", parsed['main'])
        self._set_indicator(self.cont_drogue, "DROGUE", parsed['drogue'])

        self.terminal.append(
            f"[CONT] Main: {'OK' if parsed['main'] else 'OPEN'} | "
            f"Drogue: {'OK' if parsed['drogue'] else 'OPEN'}"
        )

    def closeEvent(self, event):
        self.worker.stop()
        event.accept()
